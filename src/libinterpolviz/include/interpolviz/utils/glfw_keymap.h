/**
 * This file is part of https://github.com/adrelino/interpolation-methods
 *
 * Copyright (c) 2018 Adrian Haarbach <mail@adrian-haarbach.de>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */
#ifndef INTERPOL_GLFW_KEYMAP_H
#define INTERPOL_GLFW_KEYMAP_H

#include <unordered_map>
#include <string>

namespace interpol {

static std::string glfwGetKeyNameBetter(int key) {

    std::unordered_map<std::string, int> keymap;


//regexp:
//#define GLFW_KEY_(.{13})      (-1|\d*)
//keymap\["$1"\] = $2;

/* The unknown key */
    keymap["UNKNOWN      "] = -1;

/* Printable keys */
    keymap["SPACE        "] = 32;
    keymap["APOSTROPHE   "] = 39;  /* ' */
    keymap["COMMA        "] = 44;  /* , */
    keymap["MINUS        "] = 45;  /* - */
    keymap["PERIOD       "] = 46;  /* . */
    keymap["SLASH        "] = 47;  /* / */
    keymap["0            "] = 48;
    keymap["1            "] = 49;
    keymap["2            "] = 50;
    keymap["3            "] = 51;
    keymap["4            "] = 52;
    keymap["5            "] = 53;
    keymap["6            "] = 54;
    keymap["7            "] = 55;
    keymap["8            "] = 56;
    keymap["9            "] = 57;
    keymap["SEMICOLON    "] = 59;  /* ; */
    keymap["EQUAL        "] = 61;  /* = */
    keymap["A            "] = 65;
    keymap["B            "] = 66;
    keymap["C            "] = 67;
    keymap["D            "] = 68;
    keymap["E            "] = 69;
    keymap["F            "] = 70;
    keymap["G            "] = 71;
    keymap["H            "] = 72;
    keymap["I            "] = 73;
    keymap["J            "] = 74;
    keymap["K            "] = 75;
    keymap["L            "] = 76;
    keymap["M            "] = 77;
    keymap["N            "] = 78;
    keymap["O            "] = 79;
    keymap["P            "] = 80;
    keymap["Q            "] = 81;
    keymap["R            "] = 82;
    keymap["S            "] = 83;
    keymap["T            "] = 84;
    keymap["U            "] = 85;
    keymap["V            "] = 86;
    keymap["W            "] = 87;
    keymap["X            "] = 88;
    keymap["Y            "] = 89;
    keymap["Z            "] = 90;
    keymap["LEFT_BRACKET "] = 91;  /* [ */
    keymap["BACKSLASH    "] = 92;  /* \ */
    keymap["RIGHT_BRACKET"] = 93;  /* ] */
    keymap["GRAVE_ACCENT "] = 96;  /* ` */
    keymap["WORLD_1      "] = 161; /* non-US #1 */
    keymap["WORLD_2      "] = 162; /* non-US #2 */

/* Function keys */
    keymap["ESCAPE       "] = 256;
    keymap["ENTER        "] = 257;
    keymap["TAB          "] = 258;
    keymap["BACKSPACE    "] = 259;
    keymap["INSERT       "] = 260;
    keymap["DELETE       "] = 261;
    keymap["RIGHT        "] = 262;
    keymap["LEFT         "] = 263;
    keymap["DOWN         "] = 264;
    keymap["UP           "] = 265;
    keymap["PAGE_UP      "] = 266;
    keymap["PAGE_DOWN    "] = 267;
    keymap["HOME         "] = 268;
    keymap["END          "] = 269;
    keymap["CAPS_LOCK    "] = 280;
    keymap["SCROLL_LOCK  "] = 281;
    keymap["NUM_LOCK     "] = 282;
    keymap["PRINT_SCREEN "] = 283;
    keymap["PAUSE        "] = 284;
    keymap["F1           "] = 290;
    keymap["F2           "] = 291;
    keymap["F3           "] = 292;
    keymap["F4           "] = 293;
    keymap["F5           "] = 294;
    keymap["F6           "] = 295;
    keymap["F7           "] = 296;
    keymap["F8           "] = 297;
    keymap["F9           "] = 298;
    keymap["F10          "] = 299;
    keymap["F11          "] = 300;
    keymap["F12          "] = 301;
    keymap["F13          "] = 302;
    keymap["F14          "] = 303;
    keymap["F15          "] = 304;
    keymap["F16          "] = 305;
    keymap["F17          "] = 306;
    keymap["F18          "] = 307;
    keymap["F19          "] = 308;
    keymap["F20          "] = 309;
    keymap["F21          "] = 310;
    keymap["F22          "] = 311;
    keymap["F23          "] = 312;
    keymap["F24          "] = 313;
    keymap["F25          "] = 314;
    keymap["KP_0         "] = 320;
    keymap["KP_1         "] = 321;
    keymap["KP_2         "] = 322;
    keymap["KP_3         "] = 323;
    keymap["KP_4         "] = 324;
    keymap["KP_5         "] = 325;
    keymap["KP_6         "] = 326;
    keymap["KP_7         "] = 327;
    keymap["KP_8         "] = 328;
    keymap["KP_9         "] = 329;
    keymap["KP_DECIMAL   "] = 330;
    keymap["KP_DIVIDE    "] = 331;
    keymap["KP_MULTIPLY  "] = 332;
    keymap["KP_SUBTRACT  "] = 333;
    keymap["KP_ADD       "] = 334;
    keymap["KP_ENTER     "] = 335;
    keymap["KP_EQUAL     "] = 336;
    keymap["LEFT_SHIFT   "] = 340;
    keymap["LEFT_CONTROL "] = 341;
    keymap["LEFT_ALT     "] = 342;
    keymap["LEFT_SUPER   "] = 343;
    keymap["RIGHT_SHIFT  "] = 344;
    keymap["RIGHT_CONTROL"] = 345;
    keymap["RIGHT_ALT    "] = 346;
    keymap["RIGHT_SUPER  "] = 347;
    keymap["MENU         "] = 348;


    std::unordered_map<int, std::string> reverse_keymap;

    for(auto& pair : keymap){
        reverse_keymap[pair.second] = pair.first;
    }

    return reverse_keymap[key];
}

} // ns interpol

#endif // INTERPOL_GLFW_KEYMAP_H
