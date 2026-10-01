add_executable(physx_platform_smoke platform_smoke.cpp)
target_include_directories(physx_platform_smoke PRIVATE ${PHYSX_ROOT_DIR}/include)
target_compile_features(physx_platform_smoke PRIVATE cxx_std_14)
target_compile_definitions(physx_platform_smoke PRIVATE PX_PHYSX_STATIC_LIB
    $<$<CONFIG:debug>:_DEBUG> $<$<NOT:$<CONFIG:debug>>:NDEBUG>)
target_link_libraries(physx_platform_smoke PRIVATE PhysXExtensions PhysX PhysXCooking
    PhysXPvdSDK PhysXCommon PhysXFoundation)
add_test(NAME physx_platform_smoke COMMAND physx_platform_smoke)
