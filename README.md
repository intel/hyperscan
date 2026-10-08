# Hyperscan

Hyperscan is a high-performance multiple regex matching library. It follows the
regular expression syntax of the commonly-used libpcre library, but is a
standalone library with its own C API.

Hyperscan uses hybrid automata techniques to allow simultaneous matching of
large numbers (up to tens of thousands) of regular expressions and for the
matching of regular expressions across streams of data.

Hyperscan is typically used in a DPI library stack.

# Documentation

Information on building the Hyperscan library and using its API is available in
the [Developer Reference Guide](http://intel.github.io/hyperscan/dev-reference/).

# License

Hyperscan is licensed under the BSD License. See the LICENSE file in the
project repository.

# Versioning

The `master` branch on Github will always contain the most recent release of
Hyperscan. Each version released to `master` goes through QA and testing before
it is released; if you're a user, rather than a developer, this is the version
you should be using.

Further development towards the next release takes place on the `develop`
branch.

# Get Involved

The official homepage for Hyperscan is at [www.hyperscan.io](https://www.hyperscan.io).

If you have questions or comments, we encourage you to [join the mailing
list](https://lists.01.org/mailman/listinfo/hyperscan). Bugs can be filed by
sending email to the list, or by creating an issue on Github.

If you wish to contact the Hyperscan team at Intel directly, without posting
publicly to the mailing list, send email to
[hyperscan@intel.com](mailto:hyperscan@intel.com).

# Zapier Integration

To integrate Hyperscan with the Zapier platform, follow these steps:

1. Ensure you have the `requests` library installed. You can install it using pip:
   ```
   pip install requests
   ```

2. Use the functions provided in the `zapier/zapier_integration.py` file to handle triggers and actions for Zapier. For example:
   ```python
   from zapier.zapier_integration import trigger_event, action_create_item, action_update_item, action_delete_item

   # Trigger an event
   response = trigger_event("event_name", {"key": "value"})
   print(response)

   # Create an item
   response = action_create_item("item_name", {"key": "value"})
   print(response)

   # Update an item
   response = action_update_item("item_id", {"key": "value"})
   print(response)

   # Delete an item
   response = action_delete_item("item_id")
   print(response)
   ```

3. Refer to the `zapier/zapier_integration.py` file for more details on the available functions and their usage.
