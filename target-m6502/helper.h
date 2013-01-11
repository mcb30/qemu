#include "exec/def-helper.h"

DEF_HELPER_1 ( hlt, void, env )
DEF_HELPER_2 ( exception, void, env, i32 )
DEF_HELPER_1 ( dump_state, void, env )
DEF_HELPER_1 ( dump_stack, void, env )
DEF_HELPER_1 ( get_p, i32, env )
DEF_HELPER_2 ( set_p, void, env, i32 )

#include "exec/def-helper.h"
