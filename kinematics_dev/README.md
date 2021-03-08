# SVA Based 4D Kinematic Model
The mathematics is explaind in the PDF. 


## TODO

1. Finish each member function
2. Compare everything to MATLAB results


## Things to optimize

1. Can probably go through the declarations of eigen arrays and change some of them to pointers
2. Optimize matrix operations
3. Can sacrifice readability by replacing the matrix declarations with just slicing of main array
4. Can maintain readability by replacing matrix declarations with preprocessor define directives (is this bad practice? Yes)

```cpp
    # define rotWorld_Base transWorld_Base->block(0, 0, 3, 3);
    # define dWorld_Base transWorld_Base->block(0, 3, 3, 1);
    // instead of...
    Eigen::Vector3d dWorld_Base = transWorld_Base->block(0, 3, 3, 1);
    Eigen::Vector3d dWorld_Base = transWorld_Base->block(0, 3, 3, 1);
```