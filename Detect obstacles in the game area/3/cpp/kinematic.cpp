#include "kinematic.h"

Kinematic::Kinematic() : Thread("KinThread", .1), B(KinModel::instance().B) {}

void Kinematic::open() {
    cout << "Opening Kinematic Thread" << endl;
}

void Kinematic::step() {
    cout << "Running Thread" << endl;
}

void Kinematic::close() {
    cout << "Closing Thread" << endl;
}

void Kinematic::goHome() {
    arr q_home = {-0.103544, -0.704864, 0.490107, -0.151481, -0.265379,
                  1.57693,   -1.27666,  1.81508,  1.26323,   -0.75817,
                  0.33901,   0.985583,  1.32958,  -0.903515, -0.102393,
                  0.,        0.};

    goToJointState(q_home);
}

void Kinematic::goToJointState(arr q_home) {
    KinModel::instance().sync();

    // Make sure vacuum stays engaged
    q_home(-1) = (double)B.getGripperGrabbed("left");

    B.move({q_home}, {5.});
    B.wait();
}

arr Kinematic::getJointState(arr position, arr q_0, arr ydir) {
    rai::KinematicWorld &K = KinModel::instance().K;
    K.setJointState(q_0);

    arr y, J, Phi, PhiJ;
    arr y_vec1 = {10.}, y_vec2 = {10.}, y_vec3 = {10.}, J_vec;

    arr q, W;
    q = q_0;

    double w = rai::getParameter("w", 1e-4);
    uint n = K.getJointStateDimension();

    W.setDiag(w, n); // W is equal the Id_n matrix times scalar w

    // Clean up frame when we're done
    std::unique_ptr<rai::Frame> targetFrame(K.addObject(
        "movement_target", nullptr, rai::ST_marker, {}, {}, position, {}));

    int i = 0;
    do {
        i++;

        Phi.clear();
        PhiJ.clear();

        // Position of arm
        K.evalFeature(y, J, FS_positionDiff, {"baxterL", "movement_target"});
        Phi.append(y / 50.);
        PhiJ.append(J / 50.);

        // scalar prod zero btw Red and Blue of hand and Z of virt_obj
        K.evalFeature(y_vec1, J_vec, FS_scalarProductXZ,
                      {"baxterL", "movement_target"}); // <Red,Z> = 0
        Phi.append(y_vec1 / 100.);
        PhiJ.append(J_vec / 100.);

        K.evalFeature(y_vec2, J_vec, FS_scalarProductYZ,
                      {"baxterL", "movement_target"}); // <Green,Z> = 0
        Phi.append(y_vec2 / 100.);
        PhiJ.append(J_vec / 100.);

        K.evalFeature(y_vec3, J_vec, FS_vectorY, {"baxterL"});
        Phi.append((ydir - y_vec3) / 100.);
        PhiJ.append(J_vec / 100.);

        arr del = 0.3 * inverse(~PhiJ * PhiJ + W) * ~PhiJ * Phi;

        q -= del;

        K.setJointState(q);
    } while ((~(y) * (y))(0) + abs(y_vec2(0)) + abs(y_vec1(0)) > 0.000001 &&
             i < 10000);

    return q;
}

void Kinematic::placeToken(arr position) {}

bool Kinematic::stopRobot() {
    close();
    return true;
}

void Kinematic::toggleVacuumGripper() {
    arr q = B.getJointPositions();
    if (B.getGripperGrabbed("left")) {
        cout << "stopping vaccum gripper..." << endl;
    } else {
        cout << "starting vaccum gripper..." << endl;
        B.moveHard(q);
        rai::wait(0.1);
    }

    q(-1) = (double)!B.getGripperGrabbed("left");
    B.moveHard(q);
}

void Kinematic::liftEndeffector(double dist, arr ydir) {
    rai::KinematicWorld &K = KinModel::instance().K;
    arr y, J;
    K.evalFeature(y, J, FS_position, {"baxterL"});
    y(2) = y(2) + dist;
    arr q = getJointState(y, K.getJointState(), ydir);
    goToJointState(q);
}
