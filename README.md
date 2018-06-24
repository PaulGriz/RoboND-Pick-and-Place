>   ## Project: Kinematics Pick & Place
>
>   >   **Date:** June 24, 2018
>   >
>   >   **By:** Paul Griz
>   >
>   >   Project Submission for the Udacity Robotics Software Engineer Nanodegree   

[![Udacity - Robotics NanoDegree Program](https://s3-us-west-1.amazonaws.com/udacity-robotics/Extra+Images/RoboND_flag.png)](https://www.udacity.com/robotics)

---

For the Kuka arm to function autonomously, two main problems need to be solved:

**Forward Kinematics:**

In a Forward kinematics problem, all the joint variables (the generalized coordinates associated with the revolute and prismatic joints) are known, and the goal is to calculate the pose and orientation of the end effector in a 3D world. 

**Inverse Kinematics:**

In an Inverse kinematics problem, the position and orientation of the end effector is known and the goal is to find the joint variables. 

![](.\images\Forward-to-Inverse-Kinematics.png)

>   The figure (produced by Udacity) above illustrates the difference between Forward & Inverse Kinematics 

---

### Step 1: Completing the DH Parameter Table

To solve the Forward Kinematics & calculate the pose and orientation of the end effector, a series of homogeneous transforms must be applied between each joint's coordinate reference frame.

[![img](https://d17h27t6h515a5.cloudfront.net/topher/2017/June/5940a1d0_eq/eq.png)](https://classroom.udacity.com/nanodegrees/nd209/parts/c199593e-1e9a-4830-8e29-2c86f70f489e/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/87c52cd9-09ba-4414-bc30-24ae18277d24/concepts/c0837700-3de6-4c41-8a5d-1e25936e0cdb#)

Chaining all homogeneous transformations from the base link to the end effector is required to derive the final transformation matrix used throughout the project known as the DH parameter table.

[![alt text](https://github.com/jupidity/inverse-kinematics/raw/master/photos/originDiagram.png)](https://github.com/jupidity/inverse-kinematics/blob/master/photos/originDiagram.png)

 I used the values from the `urdf` file and the diagram (produced by Udacity) above to construct each of the diagrams below:

### T0_1:

![](.\diagrams\T0_1.PNG)

### T1_2:

![](.\diagrams\T1_2.PNG)

### T2_3:

![](.\diagrams\T2_3.PNG)

### T3_4:

![](.\diagrams\T3_4.PNG)

### T4_5

![](.\diagrams\T4_5.PNG)

### T5_6:

![](.\diagrams\T5_6.PNG)

### T6_EE:

![](.\diagrams\T6_EE.PNG)

---

### The DH parameter table:

| Links              | α    | a      | **θ**    | d     |
| ------------------ | ---- | ------ | -------- | ----- |
| **T0_1**: 0 -> 1   | 0    | 0      | θ₁       | 0.75  |
| **T1_2**: 1 -> 2   | -π/2 | 0.35   | θ₂ - π/2 | 0     |
| **T2_3**: 2 -> 3   | 0    | 1.25   | θ₃       | 0     |
| **T3_4**: 3 -> 4   | -π/2 | -0.054 | θ₄       | 1.5   |
| **T4_5**: 4 -> 5   | π/2  | 0      | θ₅       | 0     |
| **T5_6**: 5 -> 6   | -π/2 | 0      | θ₆       | 0     |
| **T6_EE**: 6 -> EE | 0    | 0      | 0        | 0.303 |

With the DH Parameter table completed, I began solving the Forward Kinematics. 

---

## Forward Kinematics

**Forward Kinematics Goal:** Calculate the pose of the end effector (EE) in a 3D world (The EE's [x,y,z]).

**Requirements:** The Joint Variables (values from the DH parameters).

**Method:** The composition of homogeneous transforms from the base link to the EE. According to the equation below:

[![img](https://d17h27t6h515a5.cloudfront.net/topher/2017/June/593eecf9_eq2/eq2.png)](https://classroom.udacity.com/nanodegrees/nd209/parts/c199593e-1e9a-4830-8e29-2c86f70f489e/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/87c52cd9-09ba-4414-bc30-24ae18277d24/concepts/c0837700-3de6-4c41-8a5d-1e25936e0cdb#) 

Each individual transform was built using the DH parameters. For every transform the DH parameters were passed into this function and the output was saved as the transformation matrix for each transform:

```python
def tranformation_matrix(alpha, a, d, q):
		tranformation_matrix = Matrix([
			[	          cos(q),           -sin(q),           0,             a],
			[  sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
			[sin(q) * sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],
			[				  0, 			    0,		    0, 			  1]
		])
		return tranformation_matrix
```

The approach of saving each transformation matrix was taken to improve performance by removing the `transformation_matrix()` call from runtime. 

[Sympy](http://www.sympy.org/en/index.html) was used to symbolically compute the theta value before being passed the values during run time. The `q1-6` values below represent **θ₁** - **θ₆**: 

```python
# Joint angle symbols
q1, q2, q3, q4, q5, q6 = symbols('q1:7')
```

The end result for these calculations were the transformation matrices below:

```python
# Homogeneous Transforms
T0_1 = Matrix([[cos(q1), -sin(q1), 0, 0],
               [sin(q1), cos(q1), 0, 0],
               [0, 0, 1, 0.75],
               [0, 0, 0, 1]])

T1_2 = Matrix([[cos(q2 - half_pi), -sin(q2 - half_pi), 0, 0.35],
               [0, 0, 1, 0],
               [-sin(q2 - half_pi), -cos(q2 - half_pi), 0, 0],
               [0, 0, 0, 1]])

T2_3 = Matrix([[cos(q3), -sin(q3), 0, 1.25],
               [sin(q3), cos(q3), 0, 0],
               [0, 0, 1, 0],
               [0, 0, 0, 1]])

T3_4 = Matrix([[cos(q4), -sin(q4), 0, -0.054],
               [0, 0, 1, 1.5],
               [-sin(q4), -cos(q4), 0, 0],
               [0, 0, 0, 1]])

T4_5 = Matrix([[cos(q5), -sin(q5), 0, 0],
               [0, 0, -1, 0],
               [sin(q5), cos(q5), 0, 0],
               [0, 0, 0, 1]])

T5_6 = Matrix([[cos(q6), -sin(q6), 0, 0],
               [0, 0, 1, 0],
               [-sin(q6), -cos(q6), 0, 0],
               [0, 0, 0, 1]])

T6_EE = Matrix([[1, 0, 0, 0],
                [0, 1, 0, 0],
                [0, 0, 1, 0.303],
                [0, 0, 0, 1]])

# From the base to EE Transformation matrix
T0_EE = (((((T0_1 * T1_2) * T2_3) * T3_4) * T4_5) * T5_6) * T6_EE
```

---

## Inverse Kinematics

**Forward Kinematics Goal:** Calculate the joint angles of the manipulator.

**Requirements:** The position and orientation (Cartesian coordinates) of the end effector.

**Method:**

There are two distinct methods for solving Inverse Kinematics problems: 

1.  A purely numerical approach requiring many iterations such as The Newton-Raphson algorithm
1.  A  “closed-form” solution (the preferred method)

Closed-form solutions have two main advantages: 

1.  They find solutions faster than numerical approaches because they do not require iteration.
2.  Developing functions to choose single possible solutions is easier than the purely numerical methods.

Closed-form solutions separate the Inverse Kinematics into independent **Position** and **Orientation** problems:

1.  **Position:** The Cartesian coordinates of the wrist center.
2.  **Orientation:** The end effector's composition of rotations. 

#### The Wrist Center's Position:

Given the `x`,`y`,`z` of a pose in 3D with the orientation in quaternions. The quaternion representations can be converted into roll, pitch, and yaw value to generate a rotation matrix between the base frame and WC  from the `roll, pitch, yaw` values. 

```python
(roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                 req.poses[x].orientation.z, req.poses[x].orientation.w])
```

A 3D rotation matrix is generated by post multiplying three elementary rotations about each of the principle axes. In this case, the rotation matrix is given by: 

```python
ROT_EE = ROT_EE.subs({'r': roll, 'p': pitch, 'y': yaw})
```

Since the Wrist Center (`WC`) is in the same location as Joint-5, the length of (Joint-5 + Joint-6) must be substracted from the Wrist Center. The `urdf` file was used to find the lengths of Joint-5 and Joint-6.

-   Joint-5 length = `0.11m`
-   Joint-6 length = `0.196m`

So, subtracting `0.303` from the `WC` is required to preform the calculations, and according to the equation below, all values needed to calculate the Cartesian coordinates of the WC are ready.

[![img](https://d17h27t6h515a5.cloudfront.net/topher/2017/May/591e52e6_image-4/image-4.png)](https://classroom.udacity.com/nanodegrees/nd209/parts/c199593e-1e9a-4830-8e29-2c86f70f489e/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/87c52cd9-09ba-4414-bc30-24ae18277d24/concepts/3bc41e14-e43d-4105-887c-8268a7402750#)  

Everything together produces the equation: 

```python
WC = EE - (0.303) * ROT_EE[:, 2]
```

Where `WC` is the `x`,`y`,`z` coordinates for the **Position** of the wrist center.

---

#### The Wrist Center's Orientation

![](.\diagrams\Orientation_Chart.PNG)

**`theta1`** is equal to the `atan2()` of the wrist center's `y` and `x`:

```python
theta1 = atan2(WC[1], WC[0])
```

>   Note: the atan2() function requires (y, x) coordinate pairs 

To find **``theta2``**:

-   `link_1` = `1.501` (from the `urdf` file)
-   `link_3` = `1.25` (from the `urdf` file)
-   `J2_5_len` is a vector pointing from `Joint-2` to ``WC`` at `Joint-5`.

`angle_c` is found by taking the `atan2` between:

1.  The Wrist Center's `z` coordinate minus the global `z` location of `Joint-2` which is `0.33 + 0.42` = `0.75`  
2.  The square root of the sum of Wrist Center's `x` & `y` coordinates squares minus the  `x` displacement between `Joint-1` - `Joint-2` (`0.35`)

```python
angle_c = atan2(WC[2] - 0.75, sqrt((WC[0]**2) + (WC[1]**2) - 0.35))
```

Angles `a` and `b` were found using the law of cosines applied to the `link_1`, `link_2`, `J2_5_len` triangle, resulting in the expressions:

```python
angle_a = acos((J2_5_len**2 + link_3**2 - link_1**2) / (2.0 * J2_5_len * link_3))

angle_b = acos((link_1**2 + link_3**2 - J2_5_len**2) / (2.0 * link_1 * link_3))
```

**`theta2`** can now be calculated by subtracting `pi/2` by `angle_a` by `angle_c`

```python
theta2 = float(half_pi - angle_a - angle_c)
```

**`theta3`** is calculated by subtracting `pi/2` by `angle_b` + `0.0373`

-   `0.0373` = `atan2(.056,1.5)` which was found using the distance between `Joint-4` and `Joint-5` in the `urdf` file and accounts for the sag in `Joint-4` of `-0.054m`

```python
theta3 = round(float(half_pi - (angle_b + 0.0373)), 3)
```

Using the homogeneous transformations from the Forward Kinematics solution, the Inverse Kinematics solution from `Joint-1` to `Joint-3` is calculated by multiplying the three Joint's homogeneous transformations:

```python
R0_3 = T0_1[0:3, 0:3] * T1_2[0:3, 0:3] * T2_3[0:3, 0:3]
```

Now, `theta1`, `theta2`, and `theta3` can be substituted for the `q1`, `q2`, and `q3` Sympy symbols in the new `R0_3` matrix:

```python
R0_3 = R0_3.evalf(5, subs={q1: theta1, q2: theta2,
                           q3: theta3}, chop=True)
```

Final step is to find a single set of Euler angles for `Joint-4`, `Joint-5`, and `Joint-6`. 

[![img](https://d17h27t6h515a5.cloudfront.net/topher/2017/May/591e52f0_image-5/image-5.png)](https://classroom.udacity.com/nanodegrees/nd209/parts/c199593e-1e9a-4830-8e29-2c86f70f489e/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/87c52cd9-09ba-4414-bc30-24ae18277d24/concepts/3bc41e14-e43d-4105-887c-8268a7402750#)

According to the equation above, `R3_6` is calculated by multiplying the inverse of `R0_3` by  `R0_EE`. So,

```python
R3_6 = R0_3.transpose() * ROT_EE
```

The `R3_6` was used to calculate `theta4`, `theta5`, and `theta6`:

```python
theta4 = atan2(R3_6[2, 2], -R3_6[0, 2])

theta5 = atan2(sqrt(R3_6[0, 2]*R3_6[0, 2] + R3_6[2, 2]*R3_6[2, 2]),
                    R3_6[1, 2])

theta6 = atan2(-R3_6[1, 1], R3_6[1, 0])
```

And we now have numeric values for all six thetas.

---

## Results

Within the simulation, the Kuka arm consistently achieves a 10/10 pick & place accuracy. Below is a screenshot of the arm consecutive placing 11 cylinders:

![](.\images\Result.jpg)

#### Possible Improvements:

While working on the project, I valued accuracy over speed of execution. This approach was taken because the majority of calculations used the [Sympy](http://www.sympy.org/en/index.html) library. If speed of execution was a priority, I would have used an alternative library, [Cython](http://cython.org/), or completed the project in C++.  However, since finding the kinematics solution was the project's main requirement, Sympy was used to complete the task.

Finally, developing a separate method/function to calculate the joint angle solutions within a single, optimal quadrant after calculating the Kinematics would improve the arm's overall speed and accuracy.

---

