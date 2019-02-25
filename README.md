<p algin="center">
    <img src="./res/wall-ed.png" style="max-width: 90%;">
</p>

"Robot do thing and we get mark. Great success." - Wall-Ed Probably


## Repo Organization
* Operations Package
  * Operations Node
  * Movement Node
  * Path Planning Library
* SLAM Package
  * RPLidar Node
  * Hector SLAM Node
  * Achilles SLAM Node
* Controls Package
  * Sensors Node
  * Motor Controls Library


## Coding Standards (for contributors)
* Please use snake_case for all naming throughout repo (variables, functions, files, directories, etc.)
* For all classes, structs and functions please follow a Doxygen Javadoc style of commenting.
   Refer to http://www.doxygen.nl/manual/docblocks.html for more info.  
   Variables and small components are not required but please leave clear documentation (better too much than too little). Short example of javadoc style for your reference:

```
/**
 *  A test class. A more elaborate class description.
 */
class Javadoc_Test
{
  public:
    /**
     * An enum.
     * More detailed enum description.
     */
    enum TEnum {
          TVal1, // Normal comment is fine
          TVal2, // Val2 comment  
          TVal3  // Val3 comment
         }

      /**
       * A constructor.
       * A more elaborate description of the constructor.
       */
      Javadoc_Test();

      /**
       * a normal member taking two arguments and returning an integer value.
       * @param a an integer argument.
       * @param s a constant character pointer.
       * @return The test results
       */
       int testMe(int a,const char *s);

      /**
       * A pure virtual member.
       * @see testMe()
       * @param c1 the first argument.
       * @param c2 the second argument.
       */
       virtual void testMeToo(char c1,char c2) = 0;

       int publicVar; // Blurb about var if necessary
};
```
* Any launch files should be placed in a "launch" folder in the root of the ros_package/catkin_ws directory. Also please use a package-relative path in launch files.  
See http://wiki.ros.org/roslaunch/XML for details on how to.
* For **catkin packages** keep all public facing headers in the include direcotry. And all private headers and source in the source space (i.e the src folder).
* DELETE BRANCHES AFTER MERGING PR
