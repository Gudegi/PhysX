// Headless CPU platform regression tests. No engine, window, CUDA or assets.
#include "PxPhysicsAPI.h"
#include "foundation/PxVecMath.h"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <initializer_list>

using namespace physx;
namespace {
void require(bool ok, const char* message) {
    if (!ok) { std::fprintf(stderr, "FAIL: %s\n", message); std::exit(1); }
}
void near(const PxVec3& actual, const PxVec3& expected, float tolerance, const char* label) {
    if (!actual.isFinite() || (actual - expected).magnitude() > tolerance) {
        std::fprintf(stderr, "%s: actual=(%g,%g,%g) expected=(%g,%g,%g)\n",
            label, actual.x, actual.y, actual.z, expected.x, expected.y, expected.z);
        require(false, label);
    }
}
struct Errors : PxErrorCallback {
    unsigned count = 0;
    void reportError(PxErrorCode::Enum code, const char* msg, const char* file, int line) override {
        std::fprintf(stderr, "PhysX %d: %s (%s:%d)\n", int(code), msg, file, line);
        if (code != PxErrorCode::eDEBUG_INFO && code != PxErrorCode::eDEBUG_WARNING
            && code != PxErrorCode::ePERF_WARNING) ++count;
    }
};
constexpr float dt = 1.f / 240.f;
void step(PxScene& scene, unsigned n = 1) {
    for (unsigned i = 0; i < n; ++i) { scene.simulate(dt); scene.fetchResults(true); }
}
void testSimd() {
    using namespace physx::aos;
    const Vec4V v = V4LoadXYZW(1.f, 2.f, 3.f, 4.f);
    alignas(16) float out[4];
    V4StoreA(V4SplatElement<0>(v), out); require(out[0] == 1.f && out[3] == 1.f, "SIMD lane 0");
    V4StoreA(V4SplatElement<1>(v), out); require(out[0] == 2.f && out[3] == 2.f, "SIMD lane 1");
    V4StoreA(V4SplatElement<2>(v), out); require(out[0] == 3.f && out[3] == 3.f, "SIMD lane 2");
    V4StoreA(V4SplatElement<3>(v), out); require(out[0] == 4.f && out[3] == 4.f, "SIMD lane 3");
    std::printf("SIMD intrinsics=%d, NEON=%d\n", COMPILE_VECTOR_INTRINSICS, PX_NEON);
}
void testArticulation(PxPhysics& physics, PxScene& scene, bool fixed, bool drive) {
    auto* art = physics.createArticulationReducedCoordinate();
    art->setArticulationFlag(PxArticulationFlag::eFIX_BASE, fixed);
    art->setArticulationFlag(PxArticulationFlag::eDISABLE_SELF_COLLISION, true);
    art->setSleepThreshold(0.f);
    art->setSolverIterationCounts(16, 4);
    auto* root = art->createLink(nullptr, PxTransform(PxVec3(0, 0, 10)));
    auto* child = art->createLink(root, PxTransform(PxVec3(0, 0, 11)));
    root->setMass(1.f); root->setMassSpaceInertiaTensor(PxVec3(1.f));
    child->setMass(2.f); child->setMassSpaceInertiaTensor(PxVec3(1.f));
    child->setCMassLocalPose(PxTransform(PxVec3(.3f, .2f, .1f)));
    root->setLinearDamping(0); root->setAngularDamping(0);
    child->setLinearDamping(0); child->setAngularDamping(0);
    auto* joint = child->getInboundJoint();
    const PxTransform childFrame(PxVec3(.1f, 0, 0), PxQuat(PxHalfPi, PxVec3(0, 1, 0)));
    joint->setParentPose(PxTransform(PxVec3(0, 0, 1)) * childFrame);
    joint->setChildPose(childFrame);
    joint->setJointType(drive ? PxArticulationJointType::eREVOLUTE : PxArticulationJointType::eFIX);
    if (drive) {
        joint->setMotion(PxArticulationAxis::eTWIST, PxArticulationMotion::eFREE);
        joint->setDriveParams(PxArticulationAxis::eTWIST,
            PxArticulationDrive(2000.f, 200.f, 100.f, PxArticulationDriveType::eFORCE));
        joint->setDriveTarget(PxArticulationAxis::eTWIST, .1f);
    }
    scene.addArticulation(*art);
    auto* cache = art->createCache();
    require(cache != nullptr, "create articulation cache");
    step(scene, drive ? 480 : 8);
    art->copyInternalStateToCache(*cache, PxArticulationCacheFlag::eLINK_INCOMING_JOINT_FORCE);
    const auto wrench = cache->linkIncomingJointForce[child->getLinkIndex()];
    near(cache->linkIncomingJointForce[root->getLinkIndex()].force, PxVec3(0), 1.e-5f, "root force");
    const PxTransform worldJoint = child->getGlobalPose() * childFrame;
    if (fixed) {
        const PxVec3 support(0, 0, 19.62f);
        const PxVec3 com = child->getGlobalPose().transform(child->getCMassLocalPose().p);
        near(worldJoint.q.rotate(wrench.force), support, .08f, "static/drive weight support");
        near(worldJoint.q.rotate(wrench.torque), (com - worldJoint.p).cross(support),
            .08f, "static/drive joint moment");
        if (drive)
            require(std::fabs(joint->getJointPosition(PxArticulationAxis::eTWIST) - .1f) < .02f,
                "drive target tracking");
        else {
            child->addForce(PxVec3(5, 0, 0));
            step(scene);
            art->copyInternalStateToCache(*cache, PxArticulationCacheFlag::eLINK_INCOMING_JOINT_FORCE);
            near(worldJoint.q.rotate(cache->linkIncomingJointForce[child->getLinkIndex()].force),
                PxVec3(-5, 0, 19.62f), .08f, "external force reaction");
        }
    } else {
        near(wrench.force, PxVec3(0), .02f, "freefall internal force");
        near(wrench.torque, PxVec3(0), .02f, "freefall internal torque");
    }
    cache->release(); art->release();
}
void testContact(PxPhysics& physics, PxScene& scene) {
    auto* material = physics.createMaterial(.7f, .6f, 0.f);
    auto* ground = PxCreatePlane(physics, PxPlane(PxVec3(0,0,1), 0), *material);
    auto* body = PxCreateDynamic(physics, PxTransform(PxVec3(0,0,2)), PxSphereGeometry(.25f), *material, 1000.f);
    scene.addActor(*ground); scene.addActor(*body);
    step(scene, 480);
    near(body->getGlobalPose().p, PxVec3(0,0,.25f), .03f, "sphere ground contact");
    body->release(); ground->release(); material->release();
}
}
int main() {
    std::printf("PhysX %u.%u.%u CPU platform smoke\n", PX_PHYSICS_VERSION_MAJOR,
        PX_PHYSICS_VERSION_MINOR, PX_PHYSICS_VERSION_BUGFIX);
    testSimd();
    PxDefaultAllocator allocator; Errors errors;
    auto* foundation = PxCreateFoundation(PX_PHYSICS_VERSION, allocator, errors);
    require(foundation != nullptr, "foundation");
    auto* physics = PxCreatePhysics(PX_PHYSICS_VERSION, *foundation, PxTolerancesScale());
    require(physics != nullptr, "physics");
    require(PxInitExtensions(*physics, nullptr), "extensions");
    auto* dispatcher = PxDefaultCpuDispatcherCreate(2);
    for (auto solver : {PxSolverType::ePGS, PxSolverType::eTGS}) {
        PxSceneDesc desc(physics->getTolerancesScale());
        desc.gravity = PxVec3(0, 0, -9.81f); desc.cpuDispatcher = dispatcher;
        desc.filterShader = PxDefaultSimulationFilterShader; desc.solverType = solver;
        auto* scene = physics->createScene(desc); require(scene != nullptr, "scene");
        testArticulation(*physics, *scene, true, false);
        testArticulation(*physics, *scene, false, false);
        testArticulation(*physics, *scene, true, true);
        testContact(*physics, *scene);
        scene->release();
        std::printf("PASS: %s static load, freefall, drive, external force, contact\n",
            solver == PxSolverType::eTGS ? "TGS" : "PGS");
    }
    dispatcher->release(); PxCloseExtensions(); physics->release(); foundation->release();
    require(errors.count == 0, "SDK error callback");
    return 0;
}
