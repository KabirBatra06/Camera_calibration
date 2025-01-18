# Camera_calibration

The goal in this project is to implement the popular Zhang’s algorithm for camera calibration.
I assume the camera to be a pin-hole camera. This implies that a complete
calibration procedure will involve estimating all the 5 intrinsic parameters and the 6 extrinsic parameters
that determine the position and orientation of the camera with respect to a reference world coordinate
system. For this I establish correspondences between image points and their world coordinates.
To this end, I use a checkerboard pattern consisting of alternating black and white
squares, as shown below. I use the corners of these squares in our calibration procedure.

# Edge Detection
For edge detection I use the inbuilt `cv2.Canny()` function. This function takes in two threshold values that control the sensitivity of edge detection. Higher values of the threshold will result in fewer and stronger edges, while lowering the values will give us more and potentially weaker edges. The output of the function is a binary image where the edges are represented by pixel values of 255 and the rest of the image has pixel values of 0.

**Threshold values**: 200 and 400

---

# Fitting lines to edges
To fit lines to edges I use the `cv2.HoughLinesP()` function. This function takes as input:

- The granularity of the distance resolution
- The granularity of the angle resolution
- The minimum number of votes a line needs to be considered
- The minimum length of the lines
- The maximum distance between lines to consider them as the same line

The output of this function is a list of coordinates that represents all the lines detected.

**Parameters**:

- Distance resolution = 1
- Angle resolution = 1 degree
- Minimum number of votes = 50
- Minimum line length = 10
- Max gap = 250

---

# Finding intersections of lines
Once I have found all the lines using the Hough lines method, I first classify each line as either **horizontal** or **vertical**:

1. Calculate the angle the line forms with the x-axis by calculating the inverse tangent of the slope of the line.
2. If the magnitude of the angle is less than 45 degrees, classify the line as horizontal; otherwise, classify it as vertical.

Next, I number both the horizontal and vertical lines based on the numerical value of their y- and x-intercepts respectively:

- Horizontal lines with the smaller y-intercept are numbered lower.
- Vertical lines with the smaller x-intercept are numbered lower.

Finally, I find the intersection of each horizontal line with each vertical line using the cartesian formula for the intersection of two lines:

$$
x = 
\frac{
\bigl[(x_{11}y_{12} - x_{12}y_{11})(x_{21}-x_{22}) - (x_{21}y_{22} - x_{22}y_{21})(x_{11}-x_{12})\bigr]
}{
\bigl[(x_{11}-x_{21})(y_{21}-y_{22}) - (x_{21}-x_{22})(y_{11}-y_{12})\bigr]
}
$$

$$
y =
\frac{
\bigl[(x_{11}y_{12} - x_{12}y_{11})(y_{21}-y_{22}) - (x_{21}y_{22} - x_{22}y_{21})(y_{11}-y_{12})\bigr]
}{
\bigl[(x_{11}-x_{21})(y_{21}-y_{22}) - (x_{21}-x_{22})(y_{11}-y_{12})\bigr]
}
$$

Where the lines are represented by the set of points $(x_{11}, y_{11}), (x_{12}, y_{12})$ and $(x_{21}, y_{21}), (x_{22}, y_{22})$.

The points $(x, y)$ are numbered based on the lines that intersect to form it. For example:

- The intersection of the first horizontal and first vertical line is the 1st point,
- The intersection of the first horizontal and second vertical line is the 2nd point,

and so on.

---

# Zhang’s Calibration Algorithm

## Finding Homographies
I begin by finding homographies between the points detected in the image and the real-world coordinates of the points in the calibration pattern. I do this using the least squares method of finding homographies.

We know that our goal is to estimate a homography between the intersection points we found and the real-world coordinates of the calibration pattern.

So, for the set of interest points $(X, X')$, we want to estimate $H$ such that 

$$
X' = H X.
$$

This implies:

$$
X' \times H X = 0
$$

Where,

$$
X' = 
\begin{bmatrix}
x' \\ 
y' \\
w'
\end{bmatrix}, 
$$

$$
\quad
X  =
\begin{bmatrix}
x \\ 
y \\
w
\end{bmatrix}, 
$$

$$
\quad
H  =
\begin{bmatrix}
h_{11} & h_{12} & h_{13} \\
h_{21} & h_{22} & h_{23} \\
h_{31} & h_{32} & h_{33}
\end{bmatrix}
\=
\begin{bmatrix}
\mathbf{h}^1{}^T \\
\mathbf{h}^2{}^T \\
\mathbf{h}^3{}^T
\end{bmatrix}
$$

For one correspondence, this vector product can be simplified into:

$$
\begin{aligned}
0 \cdot \mathbf{h}^1 - w' X^T \mathbf{h}^2 + y' X^T \mathbf{h}^3 &= 0, \\
w' X^T \mathbf{h}^1 + 0 \cdot \mathbf{h}^2 - x' X^T \mathbf{h}^3 &= 0, \\
-\,y' X^T \mathbf{h}^1 + x' X^T \mathbf{h}^2 + 0 \cdot \mathbf{h}^3 &= 0.
\end{aligned}
$$

Now, given that we have $n$ correspondences and setting $h_{33}$ to 1, we can construct the following matrices:

$$
\begin{bmatrix}
0 & 0 & 0 & -w' x & -w' y & -w' w & y' x & y' y \\
w' x & w' y & w' w & 0 & 0 & 0 & -x' x & -x' y \\
\cdot & \cdot & \cdot & \cdot & \cdot & \cdot & \cdot & \cdot \\
\cdot & \cdot & \cdot & \cdot & \cdot & \cdot & \cdot & \cdot
\end{bmatrix}
$$

$$
\begin{bmatrix}
h_{11} \\ 
h_{12} \\ 
h_{13} \\ 
h_{21} \\ 
h_{22} \\ 
h_{23} \\ 
h_{31} \\ 
h_{32}
\end{bmatrix}
\=
\begin{bmatrix}
-\,y' w \\ 
x' w \\
\cdot \\
\cdot
\end{bmatrix}
$$

This can be represented as:

$$
A \, h = b,
$$

where $A$ is a $2n \times 8$ matrix, $h$ is an 8-element vector with our unknowns, and $b$ is a $2n$-element vector.

The solution to this equation is:

$$
h = (A^T A)^{-1} A^T b.
$$

Here $(A^T A)^{-1} A^T$ is called the pseudo-inverse of matrix $A$.

Using this equation and the pseudo-inverse, we can determine our linear least squares estimation of our homography. This is done for all the images of the calibration pattern, giving us 40 homographies.

---

## Finding $\omega$
Once we have all the homographies, I then use them to estimate $\omega$, the image of the absolute conic. 

Since $\omega$ is the matrix of a conic, it must be symmetric and hence we have only 6 unknowns. We can express these unknowns as a vector:

$$
b 
\= 
\begin{bmatrix}
\omega_{11} \\ 
\omega_{12} \\ 
\omega_{22} \\ 
\omega_{13} \\ 
\omega_{23} \\ 
\omega_{33}
\end{bmatrix}.
$$

Now, to represent the equation $h_1^T \omega h_2 = 0$ as

$$
V_{12}^T b = 0
$$

and the equation $h_1^T \omega h_1 - h_2^T \omega h_2 = 0$ as

$$
(V_{11} - V_{22})^T b = 0,
$$

where,

$$
V_{ij} 
\= 
\begin{bmatrix}
h_{i1} \, h_{j1} \\
h_{i1} \, h_{j2} + h_{i2} \, h_{j1} \\
h_{i2} \, h_{j2} \\
h_{i3} \, h_{j1} + h_{i1} \, h_{j3} \\
h_{i3} \, h_{j2} + h_{i2} \, h_{j3} \\
h_{i3} \, h_{j3}
\end{bmatrix}
$$

and $h_{ij}$ is the $j$-th element in the $i$-th column of the homography $H$.

Concatenating the $V_{12}$ and $V_{11} - V_{22}$ vectors above, we create

$$
V 
\=
\begin{bmatrix}
V_{12} \\
V_{11} - V_{22}
\end{bmatrix},
$$

which is a $6 \times 2$ matrix.

Now we create a single matrix by concatenating all the $V$ matrices for all the camera positions. This matrix has the size $2n \times 6$ where $n$ is the number of camera positions.

Finally, we solve the equation

$$
\begin{bmatrix}
V_1 \\
V_2 \\
\vdots \\
V_n
\end{bmatrix}_{2n \times 6}
b \= 0
$$

using the least squares method. The solution gives us all the elements of $b$ which in turn gives us the matrix $\omega$.

---

## Finding $K$
Once we have $\omega$, we can decompose it to get $K$. We know that

$$
\omega = K^{-T} \, K^{-1}.
$$

Now, $\omega$ is homogeneous but $K$ isn’t, so we cannot solve for $K$ using element-by-element equalities. This issue of scaling is taken care of by introducing a parameter $\lambda$. We then use the following formulas to solve for the intrinsic parameters:

$$
y_0 = \frac{\omega_{12}\,\omega_{13} - \omega_{11}\,\omega_{23}}{\omega_{11}\,\omega_{22} - \omega_{12}^2},
$$

$$
\lambda = \omega_{33} - \frac{\omega_{13}^2 + y_0 \,(\omega_{12}\,\omega_{13} - \omega_{11}\,\omega_{23})}{\omega_{11}},
$$

$$
\alpha_x = \sqrt{\frac{\lambda}{\omega_{11}}},
$$

$$
\alpha_y = \sqrt{\frac{\lambda \,\omega_{11}}{\omega_{11}\,\omega_{22} - \omega_{12}^2}},
$$

$$
s = \frac{-\,\omega_{12}\,\alpha_x^2\,\alpha_y}{\lambda},
$$

$$
x_0 = \frac{s \, y_0}{\alpha_y} - \frac{\omega_{13} \,\alpha_x^2}{\lambda}.
$$

Finally, the $K$ matrix is constructed as:

$$
K
\= 
\begin{bmatrix}
\alpha_x & s      & x_0 \\
0        & \alpha_y & y_0 \\
0        & 0        & 1
\end{bmatrix}.
$$

---

## Finding Extrinsic Parameters
The extrinsic parameters for one of the camera positions vis-à-vis the $z=0$ plane is characterized by the equation

$$
K^{-1} 
\begin{bmatrix}
h_1 & h_2 & h_3
\end{bmatrix}
\=
\begin{bmatrix}
r_1 & r_2 & t
\end{bmatrix}.
$$

Here $h_i$ is the $i$-th column of the homography $H$ that we estimated before.

To ensure that the values we calculate for $R$ and $t$ are scaled properly, we enforce the orthonormality property of the $R$ matrix. This gives us that

$$
r_1 = \xi \, K^{-1} \, h_1,
$$

$$
r_2 = \xi \, K^{-1} \, h_2,
$$

$$
r_3 = \xi \, r_1 \times r_2,
$$

$$
t   = \xi \, K^{-1} \, h_3,
$$

$$
\xi = \frac{1}{\lVert K^{-1} \, h_1 \rVert}.
$$

Once we have done this, we obtain all the camera calibration parameters using Zhang’s Algorithm.

---

# Refining Calibration Parameters
When calculating the calibration parameters using Zhang’s algorithm, we minimize the **algebraic distance** in the linear least square solution. Our goal is now to minimize the **Euclidian distance** between the original points in the image plane and the points reprojected onto the image plane using our estimated parameters.

To do this, we construct a cost function that aggregates the error distances for all the points. This is done for all the camera positions.

We then minimize this cost function using **Levenberg-Marquardt (LM)**. This gives us an estimate for the camera parameters that minimizes the actual Euclidian distance between the original points in the image plane and the points reprojected onto the image plane using our estimated parameters.

### Representation of $R$ for LM
The pitfall we come across while trying to use LM is that our $R$ matrix is a $3\times 3$ matrix but has only 3 degrees of freedom. We cannot use more variables than degrees of freedom when representing an entity in any algorithmic optimization. So, we are tasked with representing the $R$ matrix as a 3-element entity.

To represent the $R$ matrix as a 3-element entity, we calculate its **Rodrigues Representation**. The Rodrigues representation of a rotation matrix is:

$$
\mathbf{w}
\=
\begin{bmatrix}
w_x \\
w_y \\
w_z
\end{bmatrix}.
$$

- The direction of the rotation is encoded in the unit vector $\mathbf{w} / \|\mathbf{w}\|$.
- The angle of rotation is encoded in the magnitude $\phi = \|\mathbf{w}\|$.

To get this representation, we first calculate:

$$
\phi = \cos^{-1} \Bigl(\frac{\text{trace}(R) - 1}{2}\Bigr).
$$

Then,

$$
\mathbf{w} = 
\frac{\phi}{2 \,\sin \phi}
\begin{bmatrix}
r_{32} - r_{23} \\
r_{13} - r_{31} \\
r_{21} - r_{12}
\end{bmatrix}.
$$

Once we have the Rodrigues representation of the rotation, we need to convert the Rodrigues representation back into the $3\times 3$ matrix representation to be able to transform our points. This is done by first representing $\mathbf{w}$ as a $3\times 3$ matrix:

$$
[\mathbf{w}]
\=
\begin{bmatrix}
0 & -w_z & w_y \\
w_z & 0 & -w_x \\
-w_y & w_x & 0
\end{bmatrix}.
$$

Then we can represent

$$
R = e^{[\mathbf{w}]} 
\= I_{3\times3} + \frac{\sin \phi}{\phi} [\mathbf{w}] + \frac{1 - \cos \phi}{\phi^2} [\mathbf{w}]^2.
$$

The ability to convert the $R$ matrix to Rodrigues representation and then back to the $R$ matrix allows us to use LM to optimize the parameters and then reconstruct the camera calibration parameters we need.

So, I first convert all the $R$ matrices into their Rodrigues representations and then append them to a list of parameters that LM will minimize. I also append all the $t$ vectors and finally the 5 values of the $K$ matrix.

These parameters are then optimized by LM, and then I reconstruct the $R$ matrix from the Rodrigues representation and obtain the optimized values of $K$. This gives me all the final optimized parameters.

---

# Plotting camera Poses
I first begin by finding all the centers of the cameras. This is done using the following formula:

$$
\mathbf{C} = -R^T \mathbf{t}.
$$

Once we have all the centers of the cameras, we then move on to finding the real-world camera axes using the equation:

$$
\mathbf{X} = R^T \mathbf{X_{\text{cam}}} + \mathbf{C}.
$$

We do this for all the camera axes below:

$$
\mathbf{X_{\text{cam}}^x} = 
\begin{bmatrix}
1 \\ 0 \\ 0
\end{bmatrix}, 
\quad
\mathbf{X_{\text{cam}}^y} = 
\begin{bmatrix}
0 \\ 1 \\ 0
\end{bmatrix},
\quad
\mathbf{X_{\text{cam}}^z} = 
\begin{bmatrix}
0 \\ 0 \\ 1
\end{bmatrix}.
$$

Once we have the center and the camera axes, we can find the plane of the camera by using the point-normal equation of a plane. We know the camera plane is normal to $\mathbf{X^z}$ and the point $\mathbf{C}$ lies on the plane:

$$
(x - c_xy - c_yz - c_z)\cdot \mathbf{X^z} = 0,
$$

where the camera center is

$$
\mathbf{C} = 
\begin{bmatrix}
c_x \\ c_y \\ c_z
\end{bmatrix}.
$$

Simplifying, we get:

$$
z 
\= 
\frac{
\mathbf{X^z}[0] (x - c_x) + \mathbf{X^z}[1] (y - c_y) + \mathbf{X^z}[2] \, c_z
}{
\mathbf{X^z}[2]
}.
$$

Now we plot the $z$ values for all the $x$ and $y$ values in a 10-pixel radius of the camera center.
