#include "hs.h"
#include <string.h>

hs_error_t hs_validate(const char* pattern, unsigned int flags, char* err_msg, int err_msg_buffer_len) {
    hs_error_t err;
    hs_compile_error_t *compile_err;
    hs_database_t *db = NULL;

    err = hs_compile(pattern, flags, HS_MODE_BLOCK, NULL, &db, &compile_err);

    if (err == HS_SUCCESS) {
        hs_free_database(db);
    } else {
        if (err_msg != NULL) {
            int err_msg_len = strlen(compile_err->message);
            strncpy(err_msg, compile_err->message, err_msg_buffer_len);

            // guarantee null termination
            if (err_msg_buffer_len > 0) {
                err_msg[err_msg_buffer_len - 1] = 0;
            }

            // if we truncated the message, add ... to indicate it was elided
            if (err_msg_buffer_len > 3 && err_msg_len > err_msg_buffer_len - 1) {
                err_msg[err_msg_buffer_len - 2] = '.';
                err_msg[err_msg_buffer_len - 3] = '.';
                err_msg[err_msg_buffer_len - 4] = '.';
            }
        }

        hs_free_compile_error(compile_err);
    }

    return err;
}
