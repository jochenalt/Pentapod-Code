#ifndef STRING_HELPER_H_
#define STRING_HELPER_H_


#include <stdio.h>
#include "math.h"
#include <string>
#include <iostream>
#include <sstream>
#include <stdint.h>
#include <fstream>

using namespace std;

std::string stringFormat(const std::string &fmt, ...);

bool fileExists(const std::string& filename);

int stringToInt (const string &str, bool& ok);
double stringToFloat (const string& str, bool&ok);
string floatToString(double x);
string floatToString(double x, int decimalPlaces);
string intToString(int x);

string boolToJSonString(bool x);
string stringToJSonString(string x);

bool jsonStringToBool (const string& str, bool&ok);

std::string stringToHex(const std::string& input);
std::string hexToString(const std::string& input);
std::string intToHex(int i);
long hexToInt(const std::string s);


string getPath(string uri);
string urlDecode(string input);
string urlEncode(const string &value);
string htmlDecode(string input);
string htmlEncode(string input);

string intToString(int x);
std::string stringFormat(const std::string &fmt, ...);
string upcase(string str);
string dncase(string str);
std::string replaceWhiteSpace(std::string s);
bool hasPrefix(string str, string prefix);

#endif
