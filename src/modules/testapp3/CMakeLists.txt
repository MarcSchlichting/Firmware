px4_add_module(
    MODULE examples__testapp3
    MAIN testapp3
    COMPILE_FLAGS
        -Wno-cast-align # TODO: fix and enable
        -Wno-address-of-packed-member # TODO: fix in c_library_v2
    SRCS
        testapp3.cpp
        helper1.cpp
    DEPENDS
    UNITY_BUILD
)
