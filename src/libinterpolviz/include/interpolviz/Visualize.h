/**
 * This file is part of https://github.com/adrelino/interpolation-methods
 *
 * Copyright (c) 2018 Adrian Haarbach <mail@adrian-haarbach.de>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */
#ifndef INTERPOL_VISUALIZE_H
#define INTERPOL_VISUALIZE_H

#include <iostream>
#include <math.h>
#include <vector>
#include <Eigen/Dense>
#include <unordered_map>
#include <map>

#include "utils/draw.h"

namespace interpol {

class Visualize {

public:
    static constexpr double maxBox[] = {1e-2, 1e-3, 1e-1, 1e-2};

    const static int FLAGS_width = 1500;
    const static int FLAGS_height = 1000;

    static bool waitKey(int key); //wait in another thread for keypress in opengl window

    //mimics opencv's Viz
    static void spin();

    static Visualize* getInstance();

    void setCentroid(Eigen::Vector3d cent);
    void setCam(std::vector<double> cam);

    static void setSelectedIndex(int i);

    //getter
           bool toggled(int key, int modifiers=0);
    static bool Toggled(int key, int modifiers=0);

           int  toggledMulti(int key, int modifiers, int toggleStateSize);
    static int  ToggledMulti(int key, int modifiers, int toggleStateSize);


    //setter
           void toggle(int key, bool value, std::string descr, std::function<void(bool)>, int modifiers=0); //with calback fun
           void toggle(int key, bool value, std::string descr, int modifiers=0);
    static void Toggle(int key, bool value, std::string descr, int modifiers=0);

    //simulate keypress/toggle
    void setToggleState(int key, int value, int modifiers=0);
    void setPressState(int key, int modifiers=0);
    void SetToggleState(int key, int value, int modifiers=0);

    void press(int key, std::string descr, std::function<void(int)>);


    int selectedFrame=255;
    int selectedOutgoingEdgeIdx=255;
    int ingoingEdgeFrame=255;

    uint64_t now;

    void setTimespan(uint64_t s, uint64_t e);

    double nowX();

    double maxScale2D;

    std::vector<std::function<void ()>> displayFunctions;  //executed in each frame
    std::vector<std::function<void ()>> displayFunctions2d;  //executed in each frame


    static void glVertex4dvQuaternion(const Eigen::Vector4d& coeffs){
        glVertex3dv(visualize::glVertex4dvQuaternion(coeffs,getInstance()->cam[7]).data());
    }

    static void glTranslate4dvQuaternion(const Eigen::Vector4d& coeffs){
        Eigen::Vector3d pos = visualize::glVertex4dvQuaternion(coeffs,getInstance()->cam[7]);
        glTranslated(pos.x(),pos.y(),pos.z());
    }

    void saveImage(std::string filename);

    static const int nKeys = 500;
    uint64_t start;
    uint64_t end;
private:

    double duration;
    //uint64_t incr;

    int refPtIdx=0;
    Visualize(); // singleton, acces via factory
    ~Visualize();

    static void main_loop();
    void shutdown();

    static const int nModifierStates = 16;

    bool keyToggle[nKeys*nModifierStates];//key toggle states
    int keyPressCounter[nKeys*nModifierStates];//for each press, increases the counter
    std::map<int,std::string> keyToggleDescr;//key toggle description
    std::map<int,std::string> keyPressDescr; //key key press description
    std::unordered_map<int,std::function<void(bool)>> functionsToggle; //functions to call on key toggle
    std::unordered_map<int,std::function<void(int)>> functionsPressed; //functions to call on key press

    int modifier;
    std::vector<double> cam;// x y z, roll pitch yaw, zoom, removedim (project 4d unit quaternion space to inside of 3d unit ball by removing 1 dimension)
    std::vector<double> cam2;// x y z

    int mouseButton;
    int moving;
    double startx, starty;

    int nbFrames = 0;
    double lastTime = 0;

    static Visualize *instance;

    GLFWwindow* window;
    
    int lastKey=-1;

    bool waitKeyInst(int key);

    void display(GLFWwindow* window);
    void keyboard(GLFWwindow* window, int key, int scancode, int action, int modifiers);
    void character(GLFWwindow* window, unsigned int codepoint);
    void mouse_button_callback(GLFWwindow* window, int button, int action, int mods);
    void cursor_position_callback(GLFWwindow* window, double x, double y);
    void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);
    void drop_callback(GLFWwindow* window, int count, const char** paths);

    static void displayW(GLFWwindow* window);
    static void keyboardW (GLFWwindow* window, int key, int scancode, int action, int modifiers);
    static void characterW(GLFWwindow* window, unsigned int codepoint);
    static void mouse_button_callbackW(GLFWwindow* window, int button, int action, int mods);
    static void cursor_position_callbackW(GLFWwindow* window, double x, double y);
    static void scroll_callbackW(GLFWwindow* window, double xoffset, double yoffset);
    static void drop_callbackW(GLFWwindow* window, int count, const char** paths);

};

} // ns interpol

#endif // INTERPOL_VISUALIZE_H

