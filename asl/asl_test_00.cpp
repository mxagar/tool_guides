/*
    See:
    https://aslze.github.io/asl-doc/index.html

    Covered here:
    - Time
    - Arithmetic, Numbers, Random
    - String
    - Containers: Array, Dict
    - Var
    - JSON
    - XML
    - Testing
    - Logger
    - File System: Directories
    - Command Line Arguments
*/

#include <iostream>
#include <string>

#include "asl/defs.h" // infinity(), nan(), sqr(), rad2deg(), deg2rad(), random(), max(), min(), clamp()
// defs includes
// - asl/time.h: sleep(), now()
// - asl/atomic.h: AtomicCount
#include "asl/Array.h" // Array
#include "asl/Map.h" // Map, Dic
#include "asl/String.h" // String
#include "asl/Var.h" // Var
#include "asl/JSON.h" // Json
#include "asl/Xml.h" // XML
#include "asl/testing.h" // Testing
#include "asl/Log.h" // Logging
#include "asl/Directory.h" // File System; It indludes File.h
#include "asl/CmdArgs.h" // Command Line Argument Paring

using namespace asl;

int main(int argc, char** argv) {

    //
    // ----- Time
    // #include "asl/time.h" or #include "asl/defs.h"

    std::cout << "--- Time: " << std::endl;
    
    double t1 = now();
    sleep(0.5);         // sleep for 500 ms
    double t2 = now();
    double elapsed = t2 - t1;  // -> around 0.5
    std::cout << "Elapsed time: " << elapsed << " seconds" << std::endl;

    //
    // ----- Arithmetic, Numbers, Random
    // #include "asl/defs.h"

    std::cout << "--- Arithmetic, Numbers, Random: " << std::endl;
    
    double angle = 10.0; // radians
    double degrees = clamp(rad2deg(angle), 0, 360);

    // Watch out: every time we execute, the value is different
    int r1 = random(256);                // get a random uniform integer between 0 and 255
    double r2 = random(-1.5, 1.5);       // get a random uniform number between -1.5 and +1.5
    double r3 = random.normal();         // get a number from a standard distribution (mean 0, std2 1)
    double r4 = random.normal(10, 0.75); // get a number from a normal distribution (mean 10, std2 0.75)
    std::cout << "Random values: " << r1 << ", " << r2 << ", " << r3 << ", " << r4 << "." << std::endl;

    //
    // ----- String
    // #include "asl/String.h"
    // Similar to JavaScript strings, byte-based UTF-8

    std::cout << "--- String: " << std::endl;

    String firstName = "John", lastName = "Doe";
    String name = firstName + " " + lastName; // use += when possible, it's faster
    
    // Unicode (UTF-8) case conversions and case-insensitive comparisons are supported.
    String babel = "Ñandú εξέλιξη жизни"; // UTF-8 string with Latin, Greek and Cyrilic text
    std::cout << std::string(babel.toUpperCase()) << std::endl; // -> "ÑANDÚ ΕΞΈΛΙΞΗ ЖИЗНИ"

    // Convenience methods    
    String filename = " MyFile.XML";
    bool ending = filename.toLowerCase().endsWith(".xml");
    String filename2 = filename.trim(); // remove whitespace before and after
    String email = "john@doe.com";
    bool at = email.contains('@');
    String path = "path7to/file";
    Array<String> parts = path.split("/"); // cut a path into its parts
    
    // UTF-8 strings can have different byte-length
    String euros = "3€";
    euros.length(); // returns 4 (bytes)
    euros.count(); // returns 2 (characters)
    euros.chars();  // returns the array [51, 8364] (the codes of '3' and '€')

    // Automatic conversions    
    String number = 14; // int to string
    Array<double> a(20);
    a[number] = number + ".5";  // -> a[14] = 14.5  string to double
    char* file = number + ".txt"; // -> "14.txt" string to const char*
    String items = array(1, 2, 3); // -> "[1,2,3]" array of ints to string

    //
    // ----- Containers: Array, Dict
    // There are more: Map, HashMap, Stack
    // #include "asl/Array.h"
    // #include "asl/Map.h"

    std::cout << "--- Containers: Array, Dict: " << std::endl;
    
    // Array: contiguous and resizable array of any type of elements
    Array<int> numbers(3);
    numbers[0] = 4;
    numbers[1] = 3;
    numbers[2] = -2;
    for(int i=0; i<numbers.length(); i++)
        std::cout << numbers[i] << std::endl;
    // Elements can be appended at the end at any moment or the array resized or cleared:
    Array<String> names;
    names << "John" << "Susan" << "David";  // -> length = 3
    names.resize(5);  // -> length = 5
    names.clear();    // -> length = 0

    // Dic: similar to python dictionaries; it's derived from template Map:
    // template <class T=String> class Dic : public Map<String, T>
    Dic<double> myDic; // value type passed
    myDic["Alice"] = 10.0;
    myDic["Bob"] = 12.5;
    for (auto& d : myDic) {
        // Apparently, String needs to be converted to std::string due to << operator
        // You could try printf() instead, as below
        std::cout << std::string(d.key) << ": " << d.value << std::endl;
    }

    //
    // ----- Var
    // #include "asl/Var.h"
    // Similar to JavaScript vars, can hold a value of one of several types

    std::cout << "--- Var: " << std::endl;

    Var z;            // z.type() = NONE
    Var i = 3;        // i.type() = INT
    Var b = 3.5;      // b.type() = NUMBER
    Var s = "x";      // s.type() = STRING
    Var t = true;     // t.type() = BOOL
    Var n = Var::NUL; // n.type() = NUL

    // If no initialization and var["string"] -> Dic
    Var var_dict;
    var_dict["k1"] = 10.0;
    // If no initialization and var[int] -> Array
    Var var_array;
    var_array[2] = -12.5;

    Var numbers_array = Var::array({1, 3, 9, -2}); // C++11
    std::cout << "numbers_array.length() = " << numbers_array.length() << std::endl;
    for(auto& x : numbers_array)  // C++11
    {
        std::cout << std::string(x.toString()) << " " << std::endl;    
    }

    // Complex object structures can be defined flexibly and contained
    // These objects are created as type Dic<T> or by adding elements to a Var with var["key"]
    Var particle = Var("name", "particle1")
                      ("x", 15.0)
                      ("y", -1.25)
                      ("visible", true)
                      ("color", Var::array({255, 0, 255}));

    // We can access elements like with Dics: particle["x"] -> 15.0
    std::cout << "particle[\"x\"] = " << double(particle["x"]) << std::endl;

    // Any Var, also complex ones, can be converted to a String
    printf("%s\n", *particle.toString()); // {color=[255,0,255],name=particle1,visible=Y,x=15,y=-1.25}

    // Iteration of elements/objects of a complex Var
    for (auto& e : particle.object())   // C++11
    {   
        printf("%s : %s\n", *e.key, *e.value.toString());
    }

    //
    // ----- JSON
    // #include "asl/JSON.h"

    String particle_json = Json::encode(particle); // Encode Var as JSON string; equivalent to JavaScript stringify()
    bool ret_json = Json::write("data.json", particle); // Writes a var to a file in JSON format
    Var particle_read = Json::read("data.json"); // Read file data.json which is in JSON format, ans save it as Var
    if (particle_read.ok())
        printf("%s\n", *particle_read.toString()); // Print read JSON file

    Var data = Json::decode("{\"x\":\"abc\", \"y\":[1,true]}"); // decode JSON from a string; equivalent to JavaScipt parse()
    // Same data could be built like this:
    // Var data = Var("x", "abc")
    //           ("y", Var::array({1, true}))
    //           ("z", 3.14);
    // We can extend data! For example, add a property z
    data["z"] = 3.14;

    //
    // ----- XML
    // #include "asl/Xml.h"
    //

    // We can define an XML object like this
    Xml html = Xml("html")
    << (Xml("head")
        << Xml("meta", Map<>("charset", "utf8"))
        << Xml("meta", Map<>("name", "author")("content", "John Doe"))
    )
    << (Xml("body")
        << Xml("h1", "Hello")
        << Xml("p", Map<>("class", "main"), "world")
    );
    // That would be equivalent to:
    // <html>
    // <head>
    //     <meta charset="utf8"/>
    //     <meta name="author" content="John Doe"/>
    // </head>
    // <body>
    //     <h1>Hello</h1>
    //     <p class="main">world</p>
    // </body>
    // </html>

    bool ret_html = Xml::write("example.xml", html); // save XML object as XML file
    Xml html_read = Xml::read("example.xml"); // read XML from a file
    String html_string = Xml::encode(html_read); // encode XML object as string
    std::cout << "HTML string: \n" << std::string(html_string) << std::endl; // output

    Xml html2 = Xml::decode(html_string);    // decode from a string

    // Access and modify values
    String text = html("body")("h1").text();  // -> "Hello"
    html("body").set("h1", "Bye"); // now it's <h1>Bye</h1>

    for (auto& meta : html("head").children("meta"))
    {
        if (meta.has("charset"))
            std::cout << std::string(meta["charset"]) << std::endl;
    }

    //
    // ----- Testing
    // #include "asl/testing.h"

    // MACROS
    // - ASL_TEST_ENABLE_MAIN(): adds testing functionality and creates a main function that runs all defined tests.
    // - ASL_TEST_ENABLE(): use if you will provide your own main function.
    // UTILITIES: there are basically 3:
    // - ASL_ASSERT, ASL_CHECK, ASL_APPROX

    // We create a file and add tests as follows:

    /*
    #include <asl/testing.h>

    ASL_TEST_ENABLE_MAIN();
    
    ASL_TEST(MyTest)
    {
        ASL_ASSERT(2 == 1); // Check that the argument is true
        ASL_CHECK(2, >, 2); // Check that the triplet argument (x, op, y) triplet is satisfied: car.numWheels() > 2
        ASL_APPROX(3.14, 3.15, 1e-9); // Check that the triplet argument (x, d, e) triplet satisfies abs(x-y) < e
    }
    */

    //
    // ----- Logger
    // #include "asl/Log.h"

    // Specify the filename where logs should be saved when exiting the program
    Log::setFile("app.log"); // write to this file (default is "log.txt")
    Log::useConsole(false);  // do not write to the console

    int mode = -1;
    // We can use ASL_LOG_ with a Level = {ERR, WARNING, INFO, DEBUG or VERBOSE}
    ASL_LOG_(WARNING, "Ignored unknown mode %i", mode);
    // Or, equivalently, specific macros with level endings _E, _W, _I, _D, _V
    ASL_LOG_W("Ignored unknown mode %i", mode);

    //
    // ----- File System
    // #include "asl/Directory.h"
    // #include "asl/File.h"

    Directory dir(".");
    for (auto& file: dir.files("*"))
    {
        if (file.lastModified() > Date::now() - 10*Date::DAY)
            std::cout << std::string(file.name()) << ", size: " << file.size() << std::endl;
    }

    // dir.files() enumerates files
    // dir.subdirs() enumerates subdirectories
    // dir.items() enumerates both

    // A File object has the following members:
    // file.path(): the full path of this item
    // file.name(): the name of the item
    // file.directory(): the full directory containing the item
    // file.size(): the file size in bytes (or 0 if it is a directory)
    // file.lastModified(): a Date indicating the last modification time
    // file.creationDate(): a Date indicating the item's creation time

    //
    // ----- Command Line Argument Parsing
    // #include "asl/CmdArgs.h"
    // Get/Parse command line arguments; simialr to the Unix getopt()

    // How it works
    // We have (1) options and (2) arguments
    // 1. Options start with '-'
    // If an option has no value, it must end with a '!'
    // 2. After all options, the list of arguments comes
    // Example:
    // convert -format jpeg -fast! -q 85 image1.png image2.bmp

    CmdArgs args(argc, argv);
    String format = args["format"]; // get value of option -format
    int quality = args("q", 90); // get value of option -q, use value 90 if not given
    bool fast = args.is("fast"); // check if value-less option is active
    // Arguments:
    // - are accessed with args[i]
    // - the length of all is args.length()
    for(int i=0; i < args.length(); ++i)
    {
        // We could carry out the functionality call here for each of the argument files
        //convertFile(args[i], format, quality, fast);
        std::cout << std::string(args[i]) << std::endl;
    }

}