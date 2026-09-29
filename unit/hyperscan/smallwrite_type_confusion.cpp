/*
 * PTK0009676: SmallWriteEngine type confusion via forged NFA type.
 *
 * runSmallWriteEngine() dispatches on nfa->type with a catch-all else branch
 * that executes the engine as Sheng. A forged type on a McClellan small-write
 * engine is therefore a type confusion leading to OOB reads.
 */

#include "config.h"

#include "hs.h"
#include "hs_runtime.h"
#include "database.h"
#include "ue2common.h"
#include "rose/rose_internal.h"
#include "nfa/nfa_internal.h"
#include "smallwrite/smallwrite_internal.h"
#include "hs_db_hmac_key.h"

#include <openssl/hmac.h>
#include <openssl/evp.h>

#include "gtest/gtest.h"

namespace {

// Serialized layout: magic(4)+version(4)+length(4)+platform(8)+hmac(32)+
// hmac_hdr(32) = 84 bytes, then bytecode.
constexpr size_t kHmacOffset = 3 * sizeof(u32) + sizeof(u64a);
constexpr size_t kHmacHdrOffset = kHmacOffset + 32;
constexpr size_t kBytecodeStart = kHmacHdrOffset + 32;

void reseal_bytecode_hmac(char *serialized, size_t) {
    // The HMAC covers exactly the `length` bytes named in the header; the
    // serialized blob is longer than that due to alignment padding.
    u32 bc_len;
    memcpy(&bc_len, serialized + 2 * sizeof(u32), sizeof(bc_len));
    u8 new_hmac[32];
    unsigned int hmac_len = 32;
    HMAC(EVP_sha256(), HS_DB_HMAC_KEY, sizeof(HS_DB_HMAC_KEY),
         reinterpret_cast<const unsigned char *>(serialized + kBytecodeStart),
         bc_len, new_hmac, &hmac_len);
    memcpy(serialized + kHmacOffset, new_hmac, sizeof(new_hmac));
}

void reseal_header_hmac(char *serialized) {
    u8 buf[20];
    memcpy(buf, serialized, sizeof(buf));
    u8 new_hmac_hdr[32];
    unsigned int hmac_len = 32;
    HMAC(EVP_sha256(), HS_DB_HMAC_KEY, sizeof(HS_DB_HMAC_KEY),
         buf, sizeof(buf), new_hmac_hdr, &hmac_len);
    memcpy(serialized + kHmacHdrOffset, new_hmac_hdr, sizeof(new_hmac_hdr));
}

// Compile a pattern that produces a small-write engine and serialize it.
bool make_smallwrite_db(char **bytes, size_t *length) {
    hs_database_t *db = nullptr;
    hs_compile_error_t *cerr = nullptr;
    if (hs_compile(".{10,20}", HS_FLAG_DOTALL, HS_MODE_BLOCK, nullptr, &db,
                   &cerr) != HS_SUCCESS) {
        if (cerr) {
            hs_free_compile_error(cerr);
        }
        return false;
    }
    if (hs_serialize_database(db, bytes, length) != HS_SUCCESS) {
        hs_free_database(db);
        return false;
    }
    hs_free_database(db);
    return true;
}

struct NFA *get_smwr_nfa(char *bytes) {
    struct RoseEngine *rose = (struct RoseEngine *)(bytes + kBytecodeStart);
    if (!rose->smallWriteOffset) {
        return nullptr;
    }
    struct SmallWriteEngine *smwr =
        (struct SmallWriteEngine *)((char *)rose + rose->smallWriteOffset);
    return (struct NFA *)((char *)smwr + sizeof(*smwr));
}

TEST(SmallwriteTypeConfusion, BaselineSmallWriteDatabaseDeserializes) {
    char *bytes = nullptr;
    size_t length = 0;
    ASSERT_TRUE(make_smallwrite_db(&bytes, &length));

    hs_database_t *db = nullptr;
    EXPECT_EQ(HS_SUCCESS, hs_deserialize_database(bytes, length, &db));
    EXPECT_TRUE(db != nullptr);
    if (db) {
        hs_free_database(db);
    }
    free(bytes);
}

TEST(SmallwriteTypeConfusion, RejectForgedSmallWriteNfaType) {
    char *bytes = nullptr;
    size_t length = 0;
    ASSERT_TRUE(make_smallwrite_db(&bytes, &length));

    struct NFA *nfa = get_smwr_nfa(bytes);
    if (!nfa) {
        free(bytes);
        return; // no small-write engine for this pattern in this build
    }
    ASSERT_TRUE(isMcClellanType(nfa->type));

    // Forge the type: anything that isn't MCCLELLAN_NFA_8/16 falls through to
    // the Sheng executor in runSmallWriteEngine().
    nfa->type = GOUGH_NFA_8;
    reseal_bytecode_hmac(bytes, length - kBytecodeStart);
    reseal_header_hmac(bytes);

    hs_database_t *db = nullptr;
    EXPECT_EQ(HS_INVALID, hs_deserialize_database(bytes, length, &db))
        << "forged small-write nfa->type was not rejected";
    EXPECT_TRUE(db == nullptr);
    if (db) {
        hs_free_database(db);
    }
    free(bytes);
}

TEST(SmallwriteTypeConfusion, RejectShengTypeOnMcClellanSmallWrite) {
    char *bytes = nullptr;
    size_t length = 0;
    ASSERT_TRUE(make_smallwrite_db(&bytes, &length));

    struct NFA *nfa = get_smwr_nfa(bytes);
    if (!nfa) {
        free(bytes);
        return;
    }

    nfa->type = SHENG_NFA;
    reseal_bytecode_hmac(bytes, length - kBytecodeStart);
    reseal_header_hmac(bytes);

    hs_database_t *db = nullptr;
    EXPECT_EQ(HS_INVALID, hs_deserialize_database(bytes, length, &db))
        << "forged SHENG_NFA type on McClellan small-write was not rejected";
    EXPECT_TRUE(db == nullptr);
    if (db) {
        hs_free_database(db);
    }
    free(bytes);
}

} // namespace
