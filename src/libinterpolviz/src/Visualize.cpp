#include "interpolviz/Visualize.h"

#include <iostream>
#include <iomanip>
#include "utils/fileutils.h"
#include "utils/glheaders.h"
#include "utils/glfw_keymap.h"

using namespace std;

namespace interpol {

GLubyte colorEdge[4]={254,122,0,14};
GLubyte colorEdgeSel[4]={122,254,0,15};

void drawText(string s, Eigen::Vector3d posMiddle){
    const unsigned char *text = (const unsigned char*) s.c_str();
    glRasterPos3dv(posMiddle.data());
//    glutBitmapString(GLUT_BITMAP_HELVETICA_10,text);
}

Visualize::~Visualize(){
    cout<<"visualize destruct"<<endl;
    shutdown();
}

void Visualize::saveImage(std::string filename) {
}

void Visualize::shutdown(){
    cout<<"visualize shutdown"<<endl;
    fileutils::saveVector<double>("cam.txt",cam);

    glfwTerminate();
    exit(0);
}

Visualize* Visualize::instance = 0;

Visualize* Visualize::getInstance(){
    if(instance == 0){
        instance = new Visualize();
    }
    return instance;
}

static void gluPerspective2( GLdouble fovy, GLdouble aspect, GLdouble near, GLdouble far )
{
    GLdouble half_height = near * tan( fovy * 0.5 *  0.01745329251 );
    GLdouble half_width = half_height * aspect;

    glFrustum( -half_width, half_width, -half_height, half_height, near, far );
}

Visualize::Visualize()
    : modifier(-1)
    , mouseButton(0)
    , moving(false)
    , maxScale2D(1.0)
    //, current_object(0)
{
    std::fill(std::begin(keyToggle), std::end(keyToggle), false);
    std::fill(std::begin(keyPressCounter), std::end(keyPressCounter), 0);

    toggle(GLFW_KEY_P,true,"poses");
    toggle(GLFW_KEY_G,true,"ground truth poses");

    /* Initialize the library */
    if (!glfwInit()) exit(-1);

//      glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
//      glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
//    //glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
//      glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);

//    glfwWindowHint(GLFW_RESIZABLE, GL_FALSE);

    // initialize the window system
    /* Create a windowed mode window and its OpenGL context */
    window = glfwCreateWindow(FLAGS_width, FLAGS_height, "GLFW PointCloud Viewer", NULL, NULL);
    if (!window){
        shutdown();
    }

    glfwSetWindowPos(window,0,0);

    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    int window_width, window_height;
    glfwGetFramebufferSize(window, &window_width, &window_height);
    std::cout<<window_width<<","<<window_height<<std::endl;
    glViewport(0, 0, window_width, window_height);

    glfwWindowHint(GLFW_SAMPLES, 4);
    glEnable(GL_MULTISAMPLE);

    glEnable( GL_POINT_SMOOTH );


    int foo=0;
    char* bla;
    //glutInit(&foo,&bla);

//    glewExperimental = GL_TRUE;
//    glewInit();

//    GLuint vertexBuffer;
//    glGenBuffers(1,&vertexBuffer);

//    printf("%u\n", vertexBuffer);

    // initialize the GL library
    // pixel storage/packing stuff
//    glPixelStorei(GL_PACK_ALIGNMENT, 1); // for glReadPixels​
//    glPixelStorei(GL_UNPACK_ALIGNMENT, 1); // for glTexImage2D​
//    glPixelZoom(1.0, -1.0);

    // enable and set colors
    glEnable(GL_COLOR_MATERIAL);


    toggle(GLFW_KEY_F9,true,"clear color: white(true) / black(false)",[](bool toggled){
        //cout<<"toogled: "<<toggled<<endl;
        if(toggled) glClearColor(1,1,1,1);
        else glClearColor(0,0,0,1);
    });




    toggle(GLFW_KEY_F10,true,"enable blending",[](bool toggled){
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        if(toggled) {
            glEnable(GL_BLEND);
        }else{
            glDisable(GL_BLEND);
        }
    });


    // enable and set depth parameters
    glEnable(GL_DEPTH_TEST);
   //glClearDepth(1.0);

    // light parameters
    //GLfloat light_pos[] = { 1.0f, 1.0f, 1.0f, 1.0f };
    GLfloat light_amb[] = { .5f, .5f, .5f, .5f};
    GLfloat light_dif[] = { .9f, 0.9f, 0.9f, 1.0f };
    GLfloat light_spec[] = { .5f, 0.5f, 0.5f, 1.0f };

//    // enable lighting
    GLfloat light_pos[] = { 1.0f, 1.0f, 1.0f, 1.0f };

    glLightfv(GL_LIGHT0, GL_POSITION, light_pos);
    glLightfv(GL_LIGHT0, GL_AMBIENT, light_amb);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, light_dif);
    glLightfv(GL_LIGHT0, GL_SPECULAR, light_spec);


    GLfloat light1_ambient[] = { 0.2, 0.2, 0.2, 1.0 };
    GLfloat light1_diffuse[] = { 1.0, 1.0, 1.0, 1.0 };
    GLfloat light1_specular[] = { 1.0, 1.0, 1.0, 1.0 };
    GLfloat light1_position[] = { -2.0, 2.0, 1.0, 1.0 };
  //  GLfloat spot_direction[] = { -1.0, -1.0, 0.0 };

    glLightfv(GL_LIGHT1, GL_AMBIENT, light1_ambient);
    glLightfv(GL_LIGHT1, GL_DIFFUSE, light1_diffuse);
    glLightfv(GL_LIGHT1, GL_SPECULAR, light1_specular);
    glLightfv(GL_LIGHT1, GL_POSITION, light1_position);
    glLightf(GL_LIGHT1, GL_CONSTANT_ATTENUATION, 1.5);
    glLightf(GL_LIGHT1, GL_LINEAR_ATTENUATION, 0.5);
    glLightf(GL_LIGHT1, GL_QUADRATIC_ATTENUATION, 0.2);

   // glLightf(GL_LIGHT1, GL_SPOT_CUTOFF, 45.0);
    //glLightfv(GL_LIGHT1, GL_SPOT_DIRECTION, spot_direction);
    //glLightf(GL_LIGHT1, GL_SPOT_EXPONENT, 2.0);



    glShadeModel(GL_SMOOTH);


    toggle(GLFW_KEY_F11,false,"enable lightning",[](bool toggled){
        if(toggled){
            glEnable(GL_LIGHTING);
            glEnable(GL_LIGHT0);
            glEnable(GL_LIGHT1);
        }else{
            glDisable(GL_LIGHTING);
            glDisable(GL_LIGHT0);
            glDisable(GL_LIGHT1);
        }
    });


  //  glDisable(GL_CULL_FACE);
//    glEnable(GL_DEPTH_TEST);
//    glDepthMask(GL_TRUE);

    toggle(GLFW_KEY_F12,false,"enable wireframe mode",[](bool toggled) {
        if(toggled)glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        else glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    });

    glfwSetMouseButtonCallback(window,mouse_button_callbackW);
    glfwSetScrollCallback(window,scroll_callbackW);
    glfwSetCursorPosCallback(window, cursor_position_callbackW);
    glfwSetDropCallback(window, drop_callbackW);
    glfwSetCharCallback(window,characterW);
    glfwSetKeyCallback(window,keyboardW);


    cam = {0,0,-5,0,0,0,-1,0}; // x y z, roll pitch yaw, zoom, removeDim
    cam2={0,0,0};

    //fileutils::loadVector<double>("cam.txt",cam);


    int keys[] = {GLFW_KEY_X,GLFW_KEY_Y,GLFW_KEY_Z,GLFW_KEY_W};

    for (int i = 0; i < 4; i++) {
        int key = keys[i];
        press(key,"quaternion dim",[this,key,i](int modifiers){
            cout<<"dim: "<<i<<"  key: "<<key<<" pressed"<<endl;
            cam[7]=i;
        });
    }

    press(GLFW_KEY_F1,"HELP",[&](int modifiers){
        cout<<"----Toggle----"<<endl;
        for(auto& pair : keyToggleDescr){
            if(pair.second.length()>0){
                cout<<glfwGetKeyNameBetter(pair.first)<<" : "<<pair.second<<endl;
            }
        }
        cout<<"----Press-----"<<endl;
        for(auto& pair : keyPressDescr){
            if(pair.second.length()>0){
                cout<<glfwGetKeyNameBetter(pair.first)<<" : "<<pair.second<<endl;
            }
        }
    });


    for(int key= GLFW_KEY_DOWN; key <= GLFW_KEY_UP; key++){
        press(key,"maxScaleChange",[this,key](int modifiers){
            if(key==GLFW_KEY_DOWN) maxScale2D -= 0.5;
            else if(key==GLFW_KEY_UP) maxScale2D += 0.5;

            cout<<"ate \trpe \ttra \trot \tmax \t maxScale2D ="<<maxScale2D<<endl;
            cout<<maxScale2D*maxBox[0]<<"\t"<<maxScale2D*maxBox[1]<<"\t"<<maxScale2D*maxBox[2]<<"\t"<<maxScale2D*maxBox[3]<<endl;
        });
    }




    //numbers
    for (int key = GLFW_KEY_1; key <= GLFW_KEY_5; key++) {
        press(key,"load/save(alt) cam",[&,key](int modifiers){
            std::string ffffname="cam_"+std::to_string(key-GLFW_KEY_0)+".txt";
            if(modifiers & GLFW_MOD_ALT){
                fileutils::saveVector<double>(ffffname,cam);
            }else{
                fileutils::loadVector<double>(ffffname,cam);
            }
        });
    }

    toggle(GLFW_KEY_O,false,"show origin");
}

void Visualize::setCentroid(Eigen::Vector3d cent) {
    Eigen::Map<Eigen::Vector3d> trans(&cam2[0]);
    trans=cent;
}

void Visualize::setCam(vector<double> c){
    cam=c;
}

void Visualize::mouse_button_callbackW(GLFWwindow* window, int button, int action, int mods){
    instance->mouse_button_callback(window,button,action,mods);
}

void Visualize::mouse_button_callback(GLFWwindow* window, int button, int action, int mods)
{
    //cout<<button<<"\t"<<action<<"\t"<<mods<<endl;

    modifier=mods;

    if (action == GLFW_PRESS) {
        mouseButton = button;
        moving = 1;
        glfwGetCursorPos(window, &startx, &starty);
    }else if (action == GLFW_RELEASE) {
        mouseButton = button;
        moving = 0;
    }
}

void Visualize::scroll_callbackW(GLFWwindow* window, double xoffset, double yoffset){
    instance->scroll_callback(window,xoffset,yoffset);
}

void Visualize::scroll_callback(GLFWwindow* window, double xoffset, double yoffset)
{
    //cout<<xoffset<<","<<yoffset<<endl;
    cam[6] += (xoffset+yoffset)*0.1;
}

void Visualize::cursor_position_callbackW(GLFWwindow* window, double x, double y){
    instance->cursor_position_callback(window,x,y);
}


void Visualize::cursor_position_callback(GLFWwindow* window, double x, double y)
{
    //cout<<x<<","<<y<<endl;

    if (moving) {
        if (modifier==GLFW_MOD_ALT) { //Panning
            if(mouseButton==GLFW_MOUSE_BUTTON_LEFT){
                cam[1]+=(y-starty)*0.001;
                cam[0]+=(x-startx)*0.001;
            }else if(mouseButton==GLFW_MOUSE_BUTTON_MIDDLE || mouseButton==GLFW_MOUSE_BUTTON_RIGHT){
                cam[2]+=((x-startx)+(y-starty))*0.001;
            }
        }else{
            if(mouseButton==GLFW_MOUSE_BUTTON_LEFT){
//                if (modifier==GLUT_ACTIVE_CTRL) {
//                    cam[1]+=(y-starty)*0.001;
//                    cam[0]+=(x-startx)*0.001;
//                }else if (modifier==GLUT_ACTIVE_ALT){
//                    cam[2]+=(x-startx)*0.001;
//                }else{
                    cam[3] = cam[3] + (x - startx);
                    cam[4] = cam[4] + (y - starty);
//                }
            }else if(mouseButton==GLFW_MOUSE_BUTTON_MIDDLE || (mouseButton==GLFW_MOUSE_BUTTON_RIGHT && modifier==GLFW_MOD_ALT)){ //if we dont have mousewheel
                int xCenter=x-(FLAGS_width/2);
                int yCenter=y-(FLAGS_height/2);
                int startxCenter=startx-(FLAGS_width/2);
                int startyCenter=starty-(FLAGS_height/2);
                cam[5] -=(atan2(yCenter,xCenter)-atan2(startyCenter,startxCenter))*(180/M_PI); //rotate object around z axis with the angle corresponding to polar coordinates of mouse displacement
                //cam[5] -= getRotationAngleApprox(xCenter-startxCenter,startyCenter-yCenter,xCenter,yCenter);
            }else if(mouseButton==GLFW_MOUSE_BUTTON_RIGHT){
                cam[6] += (((y-starty)+(x-startx))*0.1); //allows to mirror the object if zoom <0
            }
        }
        startx = x;
        starty = y;
        //glutPostRedisplay();
    }
}

void Visualize::drop_callbackW(GLFWwindow* window, int count, const char** paths){
    instance->drop_callback(window,count,paths);
}

void Visualize::drop_callback(GLFWwindow* window, int count, const char** paths)
{
//    int i;
//    frames->resize(count);
//    for (i = 0;  i < count;  i++){
//        cout<<paths[i]<<endl;
////        shared_ptr<lidarslam::LaserScanLine> currentFrame(new lidarslam::LaserScanLine(paths[i]));//loadPLY(paths[i]);
////        (*frames)[i]=currentFrame;
//    }
}

void Visualize::display(GLFWwindow* window){
    //    glClearStencil(255); //is background for mouse clicks
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective2( /* field of view in degree */ 45.0,
            /* aspect ratio */ FLAGS_width/ (FLAGS_height*1.0),
            /* Z near */ .01, /* Z far */ 100.0);
    //glFrustum(-0.1,0.1,-0.1,0.1,.01,50.0);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();




    //glTranslated(0, 0, -15);

    GLdouble dd=0.02;

    if(GLFW_PRESS==glfwGetKey(window, GLFW_KEY_W)){
        cam[2]+=dd;
    }else if(GLFW_PRESS==glfwGetKey(window, GLFW_KEY_S)){
        cam[2]-=dd;
    }

    if(GLFW_PRESS==glfwGetKey(window, GLFW_KEY_A)){
        cam[0]+=dd;
    }else if(GLFW_PRESS==glfwGetKey(window, GLFW_KEY_D)){
        cam[0]-=dd;
    }

    if(GLFW_PRESS==glfwGetKey(window, GLFW_KEY_PAGE_UP)){
        cam[1]+=dd;
    }else if(GLFW_PRESS==glfwGetKey(window, GLFW_KEY_PAGE_DOWN)){
        cam[1]-=dd;
    }


    glTranslated(cam[0],cam[1],cam[2]);
    double z = cam[6];

    float zz=-(float)z;

    if(toggled(GLFW_KEY_F11)) {
        GLfloat light_position1[] = {zz, zz, zz, 0.0};
        glLightfv(GL_LIGHT1, GL_POSITION, light_position1);
        glColor3d(0.5, 1, 0.8);
        glPointSize(20);
        glBegin(GL_POINTS);
        glVertex3fv(light_position1);
        glEnd();
    }



    glRotated(cam[5], 0.0, 0.0, 1.0);
    glRotated(cam[4], 1.0, 0.0, 0.0);
    glRotated(cam[3], 0.0, 1.0, 0.0);
    
    glScaled(z,z,z);
//    if(keyToggle['R']) drawOrigin(); //rotation center

//    glTranslated(-centroid.x(),-centroid.y(),-centroid.z());

//    glMultMatrixd(S.matrix().data());

    if(toggled(GLFW_KEY_O)) visualize::drawOrigin();

    if(toggled(GLFW_KEY_U,GLFW_MOD_SHIFT)){
        glColor4d(0.5, 0.5, 0.5, 0.8);
        glPushMatrix();
        //glScaled(0.95,0.95,0.95);
        visualize::sphere2.draw();
        glPopMatrix();
    }


    glTranslated(cam2[0],cam2[1],cam2[2]);


    if(toggled(GLFW_KEY_F11)){
        GLfloat light_position[] = { 1.0, 1.0, 1.0, .5 };
        glLightfv(GL_LIGHT0, GL_POSITION, light_position);
        glColor3d(0.5,1,0.8);
        glPointSize(20);
        glBegin(GL_POINTS);
        glVertex3fv(light_position);
        glEnd();
    }


    for(auto& f : displayFunctions){
        f();
    }
	glPopMatrix();



    glLoadIdentity();
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    glOrtho(0.0f, 1.0f/*WINDOW_WIDTH*/, 0.0f,1.0f*FLAGS_height/FLAGS_width, 0.0f, 1.0f);
    glMatrixMode(GL_MODELVIEW);
    for(auto& f : displayFunctions2d){
        f();
    }
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();



    //FPS http://www.opengl-tutorial.org/miscellaneous/an-fps-counter/
    // Measure speed
    double currentTime = glfwGetTime ();
    double nowSeconds = fmod(currentTime,duration);
    now = start+1e9*fmod(currentTime,duration);


    nbFrames++;
    if ( currentTime - lastTime >= 1.0 ){ // If last cout was more than 0.1 sec ago
        glfwSetWindowTitle (window, (std::to_string(nbFrames)+" fps. Timespan "+std::to_string((int)floor(nowSeconds)) +"-->" +std::to_string((int)ceil(nowSeconds)) +"/"+std::to_string(duration)).c_str());
        nbFrames = 0;
        lastTime += 1.0;
    }

}

void Visualize::setTimespan(uint64_t s, uint64_t e) {
    duration = (e-s)*1e-9;
    start=s;
    end=e;
    now=s;

    //cout<<"Visualise::setTimespan duration: "<<duration<<" from "<<start*1e-9<< " to "<<end*1e-9<< " s"<<endl;
/*    incr = (e-s)/1000;
    if(incr<1) incr=1;*/
}

double Visualize::nowX() {
    return (now-start)*1.0/duration;
}

void Visualize::displayW(GLFWwindow* window){
    instance->display(window);
}

void Visualize::character(GLFWwindow *window, unsigned int codepoint){
    //cout<<codepoint<<endl;
}

void Visualize::characterW(GLFWwindow *window, unsigned int codepoint){
    instance->character(window,codepoint);
}

/**
 * http://www.glfw.org/docs/latest/input_guide.html#input_key_name
 */
void Visualize::keyboard(GLFWwindow* window, int key, int scancode, int action, int modifiers)
{
    if(action != GLFW_PRESS) return;

    bool shift = modifiers & GLFW_MOD_SHIFT;
    bool ctrl = modifiers & GLFW_MOD_CONTROL;
    bool alt = modifiers & GLFW_MOD_ALT;
    bool super = modifiers & GLFW_MOD_SUPER;


    string name = glfwGetKeyNameBetter(key);

    if(modifiers) name += " "+std::to_string(modifiers);

    lastKey=key;

    bool printed=false;

    int keyHash = key+modifiers*nKeys;

    keyToggle[keyHash] = !keyToggle[keyHash];
    keyPressCounter[keyHash]++;
    if(keyToggleDescr[keyHash].length()>0){
        std::cout<<"Toggle "<<name<<" ";
        if(ctrl) cout<<" ctrl ";
        if(alt) cout<<" alt ";
        if(shift) cout<<" shift ";
        if(super) cout<<" super ";

        std::cout<<(keyToggle[keyHash] ? "on :" : "off :");
        cout<<keyToggleDescr[keyHash];
        printed=true;
    }

    if(functionsToggle.count(keyHash)){
        cout<<" togglefun "<<endl;
        functionsToggle[keyHash](keyToggle[keyHash]);
        printed=true;
    }

    if(keyPressDescr[key].length()>0){
        std::cout<<"Press: "<<name<<" ";
        if(ctrl) cout<<" ctrl ";
        if(alt) cout<<" alt ";
        if(shift) cout<<" shift ";
        if(super) cout<<" super ";

        cout<<keyPressDescr[key];
        printed=true;

    }
    if(functionsPressed.count(key)){
        cout<<" pressfun "<<endl;
        functionsPressed[key](modifiers);
        printed=true;
    }

    if(printed) std::cout<<endl;
}

void Visualize::keyboardW (GLFWwindow* window, int key, int scancode, int action, int modifiers){
    instance->keyboard(window,key,scancode,action,modifiers);
}

void Visualize::spin(){
    //cout<<"q to continue"<<endl;
    while(Visualize::waitKey(GLFW_KEY_Q) && !glfwWindowShouldClose(instance->window)){
        /* Render here */
        displayW(instance->window);

        /* Swap front and back buffers */
        glfwSwapBuffers(instance->window);

//        std::chrono::milliseconds dura( 5 );
//        std::this_thread::sleep_for( dura );

        /* Poll for and process events */
        glfwPollEvents();
    }
    if(glfwWindowShouldClose(instance->window)){
        instance->shutdown();
    }
}

void Visualize::spinLast(){
    cout<<"ESC to quit"<<endl;
    while(Visualize::waitKey(GLFW_KEY_ESCAPE) && !glfwWindowShouldClose(instance->window)){
        /* Render here */
        displayW(instance->window);

        /* Swap front and back buffers` */
        glfwSwapBuffers(instance->window);

//        std::chrono::milliseconds dura( 5 );
//        std::this_thread::sleep_for( dura );

        /* Poll for and process events */
        glfwPollEvents();
    }

   // if(glfwWindowShouldClose(instance->window)){
        instance->shutdown();
    //}

}

void Visualize::spin(int i){
    while(i-- > 0){
        /* Render here */
        displayW(instance->window);

        /* Swap front and back buffers */
        glfwSwapBuffers(instance->window);

//        std::chrono::milliseconds dura( 5 );
//        std::this_thread::sleep_for( dura );

        /* Poll for and process events */
        glfwPollEvents();
    }
    if(glfwWindowShouldClose(instance->window)){
        instance->shutdown();
    }
}

void Visualize::spinToggle(int i){
    if(getInstance()->keyToggle[GLFW_KEY_R]){
        spin(i);
    }else{
        spin();
    }
}

bool Visualize::waitKeyInst(int key){
    //cout<<"waiting for "<<key<<endl;
//    std::chrono::milliseconds dura( 10 );
    //while(true){
//        std::this_thread::sleep_for( dura );
//        setWindowFPS();
        //sleep(1);
        if(lastKey==key){
            lastKey=-1;
            //cout<<"ok, exit waiting = "<<endl;
            return false;
        }else{
            return true;
        }
    //}
}

bool Visualize::waitKey(int key){
    return getInstance()->waitKeyInst(key);
}

bool Visualize::shouldClose(){
    return glfwWindowShouldClose(getInstance()->window);
}

bool Visualize::toggled(int c, int modifiers){
    return keyToggle[c + nKeys * modifiers];
}

int Visualize::toggledMulti(int c, int modifiers, int size){
    return (keyPressCounter[c + nKeys * modifiers]) % (size+1);
}

void Visualize::toggle(int key, bool value, std::string descr, int modifiers){
    int k = key + nKeys * modifiers;
    keyToggle[k] = value;
    keyPressCounter[k] = value;
    keyToggleDescr[k] = descr;
}

void Visualize::toggle(int key, bool value, std::string descr, std::function<void (bool)> f, int modifiers){
    toggle(key,value,descr,modifiers);
    f(value);
    functionsToggle[key + nKeys * modifiers]=f;
}

void Visualize::press(int key, std::string descr, std::function<void (int)> f){
    keyPressDescr[key] = descr;
    functionsPressed[key]=f;
}

bool Visualize::Toggled(int c, int modifiers){
    return getInstance()->toggled(c,modifiers);
}

int Visualize::ToggledMulti(int c, int modifiers, int size){
    return getInstance()->toggledMulti(c,modifiers, size);
}

void Visualize::Toggle(int key, bool value, std::string descr, int modifiers){
    getInstance()->toggle(key,value,descr, modifiers);
}

void Visualize::setToggleState(int key, int value, int modifiers) {
    int k = key + nKeys * modifiers;
    keyToggle[k] = value;
    keyPressCounter[k] = value;
    if(functionsToggle.count(key + nKeys * modifiers)) functionsToggle[key + nKeys * modifiers](value);
}

void Visualize::setPressState(int key, int modifiers) {
    if(functionsPressed.count(key)) functionsPressed[key](modifiers);
}

void Visualize::SetToggleState(int key, int value, int modifers){
    getInstance()->setToggleState(key,value, modifers);
}

} // ns interpol
