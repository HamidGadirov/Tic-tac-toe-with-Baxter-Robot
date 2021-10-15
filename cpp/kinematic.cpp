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

void Kinematic::goCheat(double time){
    arr q_cheat_home = {-0.0678786, -0.790767, 0.487806, 
                        -0.202102, -0.266913, 1.55584, -1.27551, 
                        1.90444, 1.26553, -0.811476, 0.340544, 
                        -1.43274, 1.32804, -0.043335, -0.103544, 0, 0};
    arr q_cheat_1 = {-0.0678786, -0.857879, 0.488189, -0.211689, 
                    -0.266146, 1.56428, -1.27627, 2.0866, 1.26515, 
                    -1.79476, 0.340544, -1.52976, 1.32804, 
                    -0.154932, -0.104694, 0, 0};
    goToJointState(q_cheat_home,2.);
    goToJointState(q_cheat_1,1.);
    goToJointState(q_cheat_home,1.);
}

void Kinematic::goCheer(double time) {
    arr q_cheer = {-0.0678786, -0.745898, 0.698345, 0.186379, 0.0866699,
     3.04917, -3.04342, 1.40973, 1.52823, 0.0498544, 0.130005,
      1.2176, 0.870534, -1.53551, -0.192515, 0, 0};
    goToJointState(q_cheer,time);
}

void Kinematic::goHome(double time) {
    arr q_home = {-0.103544, -0.704864, 0.490107, -0.151481, -0.265379,
                  1.57693,   -1.27666,  1.81508,  1.26323,   -0.75817,
                  0.33901,   0.985583,  1.32958,  -0.903515, -0.102393,
                  0.,        0.};

    goToJointState(q_home,time);
}

void Kinematic::goToJointState(arr q_home, double time) {
    KinModel::instance().sync();

    // Make sure vacuum stays engaged
    q_home(-1) = (double)B.getGripperGrabbed("left");

    B.move({q_home}, {time});
    B.wait();
}

arr Kinematic::getJointState(arr position, arr q_0, arr ori) {
    arr ydir = ori[1];
    arr xdir = ori[0];
    rai::KinematicWorld K;
    K.addFile("./model.g");
    K.setJointState(q_0);

    bool enablealign = true;

    if(ydir(0) == 0 &&
        ydir(1) == 0 &&
        ydir(2) == 0){
        enablealign = false;
    }

    arr y, J, Phi, PhiJ;
    arr y_vec1 = {10.}, y_vec2 = {10.}, y_vec3 = {10.}, y_vec4 = {10.}, J_vec, J_vec_y, J_vec_x;

    arr q, W;
    q = q_0;

    double w = rai::getParameter("w", 1e-4);
    uint n = K.getJointStateDimension();

    W.setDiag(w, n); // W is equal the Id_n matrix times scalar w


    std::unique_ptr<rai::Frame> targetFrame(K.addObject(
        "movement_target", nullptr, rai::ST_marker, {}, {}, position, {}));

    int i = 0;
    arr sclrprody = {0.};
    arr sclrprodx = {0.};
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

        if(enablealign){
            //cout << "ydir: " << ydir << endl;
            
            K.evalFeature(y_vec3, J_vec_y, FS_vectorY, {"baxterL"});

            K.evalFeature(y_vec4, J_vec_x, FS_vectorX, {"baxterL"});

            sclrprodx = ~((~xdir*y_vec3)*(~xdir*y_vec4))*((~xdir*y_vec3)*(~xdir*y_vec4));
            sclrprody = ~((~ydir*y_vec3)*(~ydir*y_vec4))*((~ydir*y_vec3)*(~ydir*y_vec4));
            Phi.append(sclrprodx / 70.);
            Phi.append(sclrprody / 70.);

            arr Jacobian = (~(~xdir*J_vec_y)*(~xdir*y_vec4) + ~(~xdir*J_vec_x)*(~xdir*y_vec3))*2.*(~xdir*y_vec3)*(~xdir*y_vec4);
            PhiJ.append(Jacobian/70.);
            Jacobian = (~(~ydir*J_vec_y)*(~ydir*y_vec4) + ~(~ydir*J_vec_x)*(~ydir*y_vec3))*2.*(~ydir*y_vec3)*(~ydir*y_vec4);
            PhiJ.append(Jacobian/70.);

            

        }

        arr del = 0.3 * inverse(~PhiJ * PhiJ + W) * ~PhiJ * Phi;

        q -= del;

        K.setJointState(q);
    } while ((~(y) * (y))(0) + abs(y_vec2(0)) + abs(y_vec1(0)) + abs(sclrprody(0)) + abs(sclrprodx(0)) > 0.000001 &&
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

void Kinematic::liftEndeffector(double dist, arr ori, double time) {
    //arr ydir = ori[1];
    rai::KinematicWorld &K = KinModel::instance().K;
    arr y, J;
    K.evalFeature(y, J, FS_position, {"baxterL"});
    y(2) = y(2) + dist;
    arr q = getJointState(y, K.getJointState(), ori);
    goToJointState(q,time);
}
