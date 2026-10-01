# macOS CPU port based on O3DE PhysX commit 2e098c5.
# Common SDK source lists remain in the upstream parent CMake files.
if(NOT APPLE OR NOT CMAKE_OSX_ARCHITECTURES STREQUAL "arm64")
    message(FATAL_ERROR "This port currently supports macOS arm64 only")
endif()
if(NOT PX_GENERATE_STATIC_LIBRARIES OR PX_GENERATE_GPU_PROJECTS OR PX_GENERATE_GPU_PROJECTS_ONLY)
    message(FATAL_ERROR "macOS requires static CPU libraries (GPU projects disabled)")
endif()
if(PX_BUILDSNIPPETS OR PX_BUILDPVDRUNTIME)
    message(FATAL_ERROR "macOS snippets and OmniPVD runtime are not ported yet")
endif()

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-fno-rtti -fno-exceptions -ffp-contract=off
    -fvisibility=hidden -Wall -Wextra)
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g")
set(CMAKE_CXX_FLAGS_CHECKED "-O3 -g")
set(CMAKE_CXX_FLAGS_PROFILE "-O3 -g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

set(PHYSX_MAC_COMPILE_DEFS "DISABLE_CUDA_PHYSX;PX_PUBLIC_RELEASE=1;PX_PHYSX_STATIC_LIB;PX_NVTX=0;PX_SUPPORT_OMNI_PVD=0")
set(PHYSX_MAC_DEBUG_COMPILE_DEFS "_DEBUG;PX_DEBUG=1;PX_CHECKED=1;PX_SUPPORT_PVD=1")
set(PHYSX_MAC_CHECKED_COMPILE_DEFS "NDEBUG;PX_CHECKED=1;PX_SUPPORT_PVD=1")
set(PHYSX_MAC_PROFILE_COMPILE_DEFS "NDEBUG;PX_PROFILE=1;PX_SUPPORT_PVD=1")
set(PHYSX_MAC_RELEASE_COMPILE_DEFS "NDEBUG;PX_SUPPORT_PVD=0")

foreach(component PhysXFoundation LowLevel LowLevelAABB LowLevelDynamics PhysX
        PhysXCharacterKinematic PhysXCommon PhysXCooking PhysXExtensions PhysXVehicle
        SceneQuery SimulationController PhysXPvdSDK PhysXTask)
    include(${component}.cmake)
endforeach()
set(PHYSXDISTRO_LIBS PhysXFoundation PhysX PhysXCharacterKinematic PhysXPvdSDK
    PhysXCommon PhysXCooking PhysXExtensions PhysXVehicle)
install(TARGETS ${PHYSXDISTRO_LIBS} EXPORT PhysXSDK
    DESTINATION "${PX_ROOT_LIB_DIR}/$<CONFIG>")
