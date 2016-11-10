/*
 * Copyright (c) 2016, Mimetrix, Inc. All Rights Reserved.
 */


using namespace std;
#include <string>
#include <vector>
#include <set>


#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <signal.h>
#include <string.h>
#include <netinet/ip.h>
#include <netinet/tcp.h>
#include <netinet/udp.h>
#include <net/ethernet.h>
#include <arpa/inet.h>
#include <pcap.h>
#include <hs.h>


pcap_t        *gPCAP_Handle = NULL;
hs_database_t *gHS_DB = NULL;
hs_scratch_t  *gHS_Scratch = NULL;
char          *gRules_Filename = NULL;
bool           gRules_Reload = false;


/* Signal handler function for SIGINT - allow for clean shutdown */
static void Interrupt_SIGINT(int sig)
{
    
    
    if (gPCAP_Handle)
    {
        pcap_breakloop(gPCAP_Handle);
    }
} /* Interrupt_SIGINT() */


/* Signal handler function for SIGUSR1 - trigger a reload of the rules DB */
static void Interrupt_SIGUSR1(int sig)
{

    
    gRules_Reload = true;
} /* Interrupt_SIGUSR1() */


static bool Parse_Rules_File(vector<unsigned int> &p_ids,
                             vector<string> &p_pcres,
                             vector<unsigned int> &p_flags)
{
    bool          ret_val = false;
    FILE         *fp = NULL;
    char          buffer[1024];
    int           line = 0;
    char         *pos1;
    char         *pos2;
    char         *pos3;
    unsigned int  id;
    unsigned int  flags;
    
    
        // Open us for reading
    if (!(fp = fopen(gRules_Filename, "r")))
    {
        fprintf(stderr, "Failed to open '%s' for reading!\n", gRules_Filename);
        goto bail;
    }

        // Read until EOF
    while (fgets(buffer, sizeof(buffer), fp))
    {
            // Line number counter for error messages
        line++;
        
            // Skip blank lines or lines commented out
        if ((buffer[0] == '#') || (buffer[0] == '\n'))
        {
            continue;
        }

            // Format is #####:/PCRE/FLAGS\n
        if (!(pos1 = strchr(buffer, ':')))
        {
            fprintf(stderr, "Parse error on line #%d: %s", line, buffer);
            goto bail;
        }

            // Advance over :
        pos1++;

            // Get the ID
        id = atoi(buffer);

            // Check rest of the line
        if ((*pos1 != '/') ||
            (!(pos2 = strchr(pos1+1, '/'))) ||
            (!(pos3 = strchr(pos2+1, '\n'))))
        {
            fprintf(stderr, "Parse error on line #%d: %s", line, buffer);
            goto bail;
        }

            // Advance over '/'
        pos1++;

            // Terminate PCRE and advance over '/'
        *pos2 = '\0';
        pos2++;

            // Terimate flags
        *pos3 = '\0';

            // Build up the flags
        flags = 0;
        while (*pos2)
        {
                // Parse legal flags
            switch (*pos2)
            {
            case 'i':
                flags |= HS_FLAG_CASELESS;
                break;

            case 'm':
                flags |= HS_FLAG_MULTILINE;
                break;

            case 's':
                flags |= HS_FLAG_DOTALL;
                break;

            case 'H':
                flags |= HS_FLAG_SINGLEMATCH;
                break;

            case 'V':
                flags |= HS_FLAG_ALLOWEMPTY;
                break;

            case '8':
                flags |= HS_FLAG_UTF8;
                break;

            case 'W':
                flags |= HS_FLAG_UCP;
                break;

            default:
                fprintf(stderr, "Parse error on line #%d, flag: '%c'\n", line,
                        *pos2);
                goto bail;
            }

                // Advance flag pointer
            pos2++;
        }

            // Squirl away the line's contents
        p_ids.push_back(id);
        p_pcres.push_back(pos1);
        p_flags.push_back(flags);
    }

        // If we get here, all is well
    ret_val = true;
  bail:
        // Cleanup
    if (fp)
    {
        fclose(fp);
    }

        // Return our code
    return(ret_val);
} /* Parse_Rules_File() */


/* Load the rules file from disk and swap it to active if we parse successfully */
static bool Rules_Load(void)
{
    vector<unsigned int>  ids;
    vector<string>        pcres;
    vector<const char *>  pcre_cstrs;
    vector<unsigned int>  flags;
    hs_database_t        *db = NULL;
    hs_compile_error_t   *compile_err;
    

        // Parse the file
    if (!Parse_Rules_File(ids, pcres, flags))
    {
        return(false);
    }

        // Swaparoo from string -> c_str (note: this is safe because we keep
        // around the pcres vector until we're done so memory isn't free'd
    for (const auto &pcre : pcres)
    {
        pcre_cstrs.push_back(pcre.c_str());
    }

        // Build the hyperscan DB, if we succeed, replace the active one (if any)
    if (hs_compile_multi(pcre_cstrs.data(), flags.data(), ids.data(),
                         pcre_cstrs.size(), HS_MODE_BLOCK, NULL, &db,
                         &compile_err) == HS_SUCCESS)
    {
            // Delete old scratch if any
        if (gHS_Scratch)
        {
            hs_free_scratch(gHS_Scratch);
        }

            // Delete old DB if any
        if (gHS_DB)
        {
            hs_free_database(gHS_DB);
        }

            // Squirl away
        gHS_DB = db;
        gHS_Scratch = NULL;

            // Get us scratch space
        if (hs_alloc_scratch(gHS_DB, &gHS_Scratch) != HS_SUCCESS)
        {
            fprintf(stderr, "Failed to allocate memory\n");
            exit(-1);
        }
    }
    else
    {
            // We barfed, let us know why...
        if (compile_err->expression < 0)
        {
            fprintf(stderr, "Hyperscan compiler failed with: %s\n",
                    compile_err->message);
        }
        else
        {
            fprintf(stderr, "Error in rules file, line %d, '%s'\n",
                    compile_err->expression, pcre_cstrs[compile_err->expression]);
        }

            // Clean up error
        hs_free_compile_error(compile_err);

            // Bail
        return(false);
    }

        // If we get here, all is well
    return(true);
} /* Rules_Load() */


/* Callback function on Hyperscan hs_match() matches */
static int HS_CB(unsigned int p_id,
                 unsigned long long p_from,
                 unsigned long long p_to,
                 unsigned int p_flags,
                 void *p_ctx)
{
    set<unsigned int> *matches = (set<unsigned int> *)p_ctx;
    

        // Save that we hit this ID (set so duplicates are suppressed)
    matches->insert(p_id);
    
    return(0);
} /* HS_CB() */


/*
 * Given a raw packet, advance to the payload if IPv4 and return true,
 * return false otherwise and don't bother with it
 */
static bool Find_Packet_Payload(const unsigned char **p_packet,
                                unsigned int *p_length,
                                const unsigned char **p_l3,
                                const unsigned char **p_l4)
{
    register unsigned int        local_length = *p_length;
    register const ether_header *l2 = (const ether_header *)*p_packet;
    register uint16_t            proto;
    register const ip           *l3;
    register unsigned int        l3_len;
    register unsigned int        l4_len;
    

        // Check if we have enough data for the smallest L2/L3/L4 packet
        // we care about...
    if (local_length < (sizeof(ether_header)+sizeof(ip)+sizeof(udphdr)))
    {
        return(false);
    }

        // Absorb the base ethernet header
    local_length -= sizeof(ether_header);

        // Skip 802.1q VLAN tags
    while (((proto = ntohs(l2->ether_type)) == 0x8100) &&
           (local_length >= 4))
    {
        l2 = (const ether_header *)((const unsigned char *)l2+ 4);
        local_length -= 4;
    }

        // See if we consumed everything stripping tags (unlikely)
    if (local_length == 0)
    {
        return(false);
    }
    
        // Skip non-IPv4 (cached from loop above)
        // XXX/TODO/FIXME - Extension could do the offset calculation for
        // other protocols too (ARP, IPv6, LLC/SNAP, etc), for now, just bail
    if (proto != ETHERTYPE_IP)
    {
        return(false);
    }

        // Now set the IPv4 header location
    l3 = (const ip *)((const unsigned char *)l2 + sizeof(ether_header));
    
        // Ignore packets that aren't IPv4 (shouldn't happen)
    if (l3->ip_v != 4)
    {
        return(false);
    }

        // Calculate IPv4 header length
    l3_len = l3->ip_hl * 4;

        // Make sure we have enough data for L3 header
    if (local_length < l3_len)
    {
        return(false);
    }

        // Absorb the IPv4 header
    local_length -= l3_len;

        // Figure out L4 header length based on protocol
    switch (l3->ip_p)
    {
    case IPPROTO_TCP:
        l4_len = ((const tcphdr *)((const unsigned char *)l3 + l3_len))->th_off * 4;
        break;
    case IPPROTO_UDP:
        l4_len = sizeof(udphdr);
        break;
    default:
            // XXX/TODO/FIXME - could support other exotic protocols as well...
        return(false);
    }

        // Make sure we have enough data for L4 header
    if (local_length < l4_len)
    {
        return(false);
    }

        // Set start of payload and payload length
    *p_packet = (const unsigned char *)((const unsigned char *)l3 + l3_len + l4_len);
    *p_length = local_length - l4_len;
    *p_l3 = (const unsigned char *)l3;
    *p_l4 = (const unsigned char *)l3 + l3_len;

        // Only return true if there is actual data to search
    return(*p_length != 0);
} /* Find_Packet_Payload() */


/* Callback function from libpcap pcap_loop() for each packet received */
static void PCAP_CB(unsigned char *p_arg,
                    const struct pcap_pkthdr *p_header,
                    const unsigned char *p_packet)
{
    const unsigned char       *data = (const unsigned char *)p_packet;
        //const struct ether_header *l2 = (const struct ether_header *)p_packet;
    const struct ip           *l3 = NULL;
    const struct udphdr       *l4 = NULL;
    unsigned int               length = p_header->len;
    set<unsigned int>          matches;
    hs_error_t                 err;
    bool                       first_time = true;
    char                       sip[64];
    char                       dip[64];
    

        // First check if we were signaled to reload our rules
    if (gRules_Reload)
    {
        gRules_Reload = false;
        Rules_Load();
    }

        // Find payload location
    if (Find_Packet_Payload(&data, &length, (const unsigned char **)&l3,
                            (const unsigned char **)&l4))
    {
            // Valid packet to scan
        if ((err = hs_scan(gHS_DB, (const char *)data, length, 0,
                           gHS_Scratch, HS_CB, &matches)) != HS_SUCCESS)
        {
                /* Errr.....WTF? */
            fprintf(stderr, "Hyperscan error: %d\n", err);
        }
        else if (!matches.empty())
        {
                // Only bother if we matched 1 or more terms
            inet_ntop(AF_INET, &l3->ip_src, sip, sizeof(sip));
            inet_ntop(AF_INET, &l3->ip_dst, dip, sizeof(dip));
                // NOTE: SPORT/DPORT match in UDP and TCP, so just fudge this
            fprintf(stdout, "{ \"Hyperscan\": { \"SourceIP\": \"%s\", \"DestinationIP\": \"%s\", \"SourcePort\": \"%u\", \"DestinationPort\": \"%u\", \"Protocol\": \"%s\", \"MatchRules\": [", sip, dip, ntohs(l4->source), ntohs(l4->dest), (l3->ip_p == IPPROTO_TCP) ? "TCP" : "UDP");
            for (const auto &match : matches)
            {
                if (first_time)
                {
                    first_time = false;
                }
                else
                {
                    fprintf(stdout, ", ");
                }
                fprintf(stdout, "\"%u\"", match);
            }
            fprintf(stdout, "] } }\n");
        }
    }
} /* PCAP_CB() */


int main(int argc,
         char **argv)
{
    char   *interface = NULL;
    int     opt;
    char    err[PCAP_ERRBUF_SIZE];
    

        // Need to be root to promiscuous capture
    if (getuid())
    {
        fprintf(stderr, "Must be root to run!\n");
        exit(-1);
    }
    
    while ((opt = getopt(argc, argv, "i:r:h?")) != -1)
    {
        switch (opt)
        {
        case 'i': /* Interface */
            if (interface)
            {
                fprintf(stderr, "Duplicate interface specifiers not allowed!\n");
                exit(-1);
            }
            interface = strdup(optarg);
            break;

        case 'r': /* Rules */
            if (gRules_Filename)
            {
                fprintf(stderr, "Duplicate rules files specifiers not allowed!\n");
                exit(-1);
            }
            gRules_Filename = strdup(optarg);
            break;
            
        case 'h': /* Help */
        case '?': /* Help */
        default:
            fprintf(stderr, "Usage: %s [-i <interface_name>] -r <rules_file>\n",
                    argv[0]);
            exit(-1);
        }
    }

        // Cannot proceed without a rules file
    if (!gRules_Filename)
    {
        fprintf(stderr, "Rules file must be specified to run!\n");
        exit(-1);
    }

        // Load our rules
    if (!Rules_Load())
    {
        fprintf(stderr, "Failed to load the rules database, see previous errors!\n");
        exit(-1);
    }

        // Register SIGUSR1 to cause a reload of the Rules file
    if (signal(SIGUSR1, Interrupt_SIGUSR1) == SIG_ERR)
    {
        fprintf(stderr, "Failed to register SIGUSR1 handler\n");
        exit(-1);
    }

        // If we weren't given an interface, grab PCAP's default
    if (!interface)
    {
        if (!(interface = strdup(pcap_lookupdev(err))))
        {
            fprintf(stderr, "PCAP inteface lookup error: %s\n", err);
            exit(-1);
        }
    }

        // Open up the interface for promiscuous capture
    if (!(gPCAP_Handle = pcap_open_live(interface, 1600, 1, 10, err)))
    {
        fprintf(stderr, "PCAP live capture error: %s\n", err);
        exit(-1);
    }

        // Register SIGINT to interrupt the pcap_loop() call to follow
    if (signal(SIGINT, Interrupt_SIGINT) == SIG_ERR)
    {
        fprintf(stderr, "Failed to register SIGINT handler\n");
        exit(-1);
    }
    
        // Process packets until we're interrupted by SIGINT...
    pcap_loop(gPCAP_Handle, 0, PCAP_CB, NULL);
    
        // Cleanup
    hs_free_database(gHS_DB);
    free(interface);
    free(gRules_Filename);

        // Exit stage left...
    return(0);
} /* main() */
