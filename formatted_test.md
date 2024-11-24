# sixAxSensFusion
Sensor fusion algorithms for 6-axis IMU gait analysis

# Introduction to Sensor Fusion

Gait analysis—studying how people walk—is an important task in healthcare, sports, and rehabilitation. It helps us spot walking issues, track recovery progress, and even design better treatments for conditions like stroke or Parkinson's. One great tool for this is the 6-axis Inertial Measurement Unit (IMU). These devices combine a 3-axis accelerometer and a 3-axis gyroscope, and are very handy for collecting data about movement. You can stick them on a shoe, strap them to a leg, or embed them in a wearable to track how someone moves.

But here's the catch: accelerometers and gyroscopes, while powerful, have their downsides. Accelerometers measure linear acceleration and can estimate how far someone moves or how fast, but they're noisy and can be thrown off by quick jolts or vibrations. Gyroscopes, which measure rotational velocity, are great for capturing smooth and precise turning motions, like the angle of a foot during a step. However, they tend to drift over time, leading to small errors that snowball into big ones.

This is where sensor fusion has a lot to offer. By combining data from both sensors, fusion algorithms can balance the strengths of each to help compensate for the weaknesses of the other. Gyroscope data can stabilize noisy accelerometer readings, and accelerometer data can reduce gyroscope drift. The sensor fusion algorithms discussed in this paper help this combination work smoothly, producing clean, reliable data about how someone walks.

Benefits of accurate gait analysis include helping to predict falls in older adults, tracking how well a patient is recovering from surgery, or even fine-tuning athletic performance. Without sensor fusion, IMU data would be too messy or unreliable to perform these functions. With this little bit of background out of the way, let's dive deeper into how sensor fusion works in the context of the 6-axis IMU.
# Defining our Measurement Variable

When using an IMU for gait analysis, we would like to use the IMU's measurements to calculate heel-strike, toe-off, and stride length (and perhaps we'll add toe-down and heel-off if we're feeling ambitious). At any given time $k$, the IMU will give us accelerometer data along its three local axes. We can think of this accereration data as a vector $\mathbf a^\text{local}$, where at time $k$, we have

$$\mathbf a^{\text{local}}_k = \left[a^{\text{pitch}}_k, a^{\text{roll}}_k, a^{\text{yaw}}_k\right]^T,$$

It will also give us rotational velocity along these local axes which we can write as

$$\boldsymbol\omega^{local}_k = \left[\omega^{\text{pitch}}_k, \omega^{\text{roll}}_k, \omega^{\text{yaw}}_k\right]^T.$$

The superscript $T$ here denotes the transpose, because we will want these as column vectors later. Putting these together, we can think of our measurements as being represented by a variable $\mathbf z$, where at time $k$ the IMU gives us the reading

$$\mathbf z_k = \left[a^{\text{pitch}}_k, a^{\text{roll}}_k, a^{\text{yaw}}_k, \omega^{\text{pitch}}_k, \omega^{\text{roll}}_k, \omega^{\text{yaw}}_k\right]^T.$$


It's important to keep in mind that these measurements are with respect to the local frame of the IMU, and not the world frame.
# Defining our State Variable

In order to determine when and how gait events happen, we would need to know the IMU's position and orientation in world frame axes, such as north($N$)-east($E$)-down($D$) axes. Additionally, it would be nice to have the IMU's velocity and acceleration in the world frame. To visualize this, we could assign variables to position, linear velocity, linear acceleration, orientation, and angular velocity, like this:

$$\begin{align*}
\mathbf p^{\text{world}}_k &= \left[p^{\text{N}}_k, p^{\text{E}}_k, p^{\text{D}}_k\right]^T, \\
\mathbf v^{\text{world}}_k &= \left[v^{\text{N}}_k, v^{\text{E}}_k, v^{\text{D}}_k\right]^T, \\
\mathbf a^{\text{world}}_k &= \left[a^{\text{N}}_k, a^{\text{E}}_k, a^{\text{D}}_k\right]^T, \\
\mathbf q^{\text{world}}_k &= \left[q^0_k, q^1_k, q^2_k, q^3_k\right]^T, \\
\boldsymbol\omega^{\text{world}}_k &= \left[\omega^{\text{N}}_k, \omega^{\text{E}}_k, \omega^{\text{D}}_k\right]^T. \\
\end{align*}$$

Here, $\mathbf q^{\text{world}}_k$ is a vector representation of the quaternion $\left[q^0_k + i\left(q^1_k\right) + j\left(q^2_k\right) + k\left(q^3_k\right)\right]$. We use quaternions rather than matricies to represent orientation because they let us update our orientation much more easily using the quaternion update function

$$\mathbf q_{k+1} = \mathbf q_k+\frac12dt\cdot\mathbf q_k\otimes\begin{bmatrix}0 \\ \omega^{\text{N}}_k \\ \omega^{\text{E}}_k \\ \omega^{\text{D}}_k\end{bmatrix}.$$


Putting these together, we can think of our system state (at least the parts of it we're interested in) as being represented by a variable $\mathbf x$, where at time $k$ we estimate that its properties are

$$\mathbf x_k = \left[p^{\text{N}}_k, p^{\text{E}}_k, p^{\text{D}}_k, v^{\text{N}}_k, v^{\text{E}}_k, v^{\text{D}}_k, a^{\text{N}}_k, a^{\text{E}}_k, a^{\text{D}}_k, q^0_k, q^1_k, q^2_k, q^3_k, \omega^{\text{N}}_k, \omega^{\text{E}}_k, \omega^{\text{D}}_k\right]^T.$$

# Translating Between Local and World Axes

Our goal is to use our local pitch-roll-yaw coordinate system measurements to estimate the system state in terms of the global coordinate system. A dificulty with calculating acceleration in this manner is that the direction of gravity will change as our local axes rotate, and our accelerometers will not be able to distinguish these orientation changes from actual world acceleration changes. In this section, we take advantage of our information on orientation in order to remedy the issue.

At a given time $k$, we will have the IMU's orientation stored as a quaterion $\mathbf q_k$ that tells us how to rotate from a "neutral" orientation to IMU's current orientation. In order to use this information to calculate the component of acceleration which is gravity, we could stick to our quaternion guns and derive the proper sequence of quaternion multiplication, but since we are dealing with a "static" orientation here (since we are discretizing the problem, we treat $\mathbf q_k$ as constant for the duration of this specific time step) and do not need to interpolate between states, it will be more efficient to use matricies.

The rotation matrix $\mathbf C_k$, defined as

$$\mathbf C_k = \begin{bmatrix}
1 - 2\big((q^2_k)^2 + (q^3_k)^2\big) & 2\big(q^1_k q^2_k - q^0_k q^3_k\big) & 2\big(q^1_k q^3_k + q^0_k q^2_k\big) \\
2\big(q^1_k q^2_k + q^0_k q^3_k\big) & 1 - 2\big((q^1_k)^2 + (q^3_k)^2\big) & 2\big(q^2_k q^3_k - q^0_k q^1_k\big) \\
2\big(q^1_k q^3_k - q^0_k q^2_k\big) & 2\big(q^2_k q^3_k + q^0_k q^1_k\big) & 1 - 2\big((q^1_k)^2 + (q^2_k)^2\big)
\end{bmatrix},$$

rotates a vector from the local frame to the world frame. In other words, we have

$$\begin{align*}
\mathbf a^{\text{world}}_k = \mathbf C_k \cdot \mathbf a^{\text{local}}_k, \\
\boldsymbol\omega^{\text{world}}_k = \mathbf C_k \cdot \boldsymbol\omega^{\text{local}}_k.
\end{align*}$$

Furthermore, since $\mathbf C_k$ is an orthogonal matrix, its inverse is equal to its transpose $\mathbf C^T_k$, meaning that

$$\begin{align*}
\mathbf a^{\text{local}}_k = \mathbf C^T_k \cdot \mathbf a^{\text{world}}_k, \\
\boldsymbol\omega^{\text{local}}_k = \mathbf C^T_k \cdot \boldsymbol\omega^{\text{world}}_k.
\end{align*}$$

Because of this, we can calculate world frame acceleration from our local measurements in a way that accounts for gravity. If we are measuring acceleration m/s^2, and at time $k$ our sensor is stationary and aligned with the $N$-$E$-$D$ axes, it should read

$$\mathbf a^{\text{local}}_k = \begin{bmatrix}a^{\text{N}}_k \\ a^{\text{E}}_k \\ a^{\text{D}}_k\end{bmatrix} = \begin{bmatrix}0 \\ 0 \\ 9.8\end{bmatrix}.$$

Therefore, a stationary sensor with any given pitch-roll-yaw axes should read

$$\mathbf a^{\text{local}}_k = \mathbf C^T_k \begin{bmatrix}0 \\ 0 \\ 9.8\end{bmatrix}.$$

By extension, any deviation from this value means that the sensor is actually accelerating in the world frame, so at any time $k$ our sensor should read

$$\mathbf a^{\text{local}}_k = \mathbf C^T_k \left(\mathbf a^{\text{world}}_k + \begin{bmatrix}0 \\ 0 \\ 9.8\end{bmatrix}\right).$$

This looks like exactly what we need! To make things more concise, we will add $\mathbf a^{\text{world}}$ and gravity together into one vector, and write

$$\mathbf a^{\text{local}}_k = \mathbf C^T_k \begin{bmatrix}a^{\text{N}}_k \\ a^{\text{E}}_k \\ a^{\text{D}}_k + 9.8\end{bmatrix}.$$


Now that we've defined our problem and seen a bit of how our measurment and state variables relate to each other, it's time to build a sensor fusion algorith to estimate state from measurements. We have a variety of options such as complemetary filter, madgwick filter, mahony filter, kalman filter, etc....
# Kalman Filter

The kalman filter estimates an nx1 column vector state variable ($\mathbf x$), based on some mx1 column vector measurement ($\mathbf z$), using a system model:

$$\begin{align*}
\text{state transition matrix} &: \mathbf A  &(& \text{nxn matrix}), \\
\text{process noise covariance} &: \mathbf Q  &(& \text{nxn diagonal matrix}), \\
\text{measurement covariance} &: \mathbf C  &(& \text{mxm matrix}), \\
\text{measurement model matrix} &: \mathbf H  &(& \text{mxn matrix}).
\end{align*}$$


After the system model has been set, there are five steps of the simple kalman filter:

0. <u>Set initial values</u>

$$\begin{align*}
\mathbf x_0 &= \text{initial state} &(& \text{nx1 column vector}), \\
\mathbf P_0 &= \text{initial error covariance} &(& \text{nxn matrix}).
\end{align*}$$


1. <u>Predict state and error covariance:</u>

$$\begin{align*}
\mathbf{\bar x}_k &= \mathbf A \mathbf x_{k-1}, \\
\mathbf{\bar P}_k &= \mathbf A \mathbf P_{k-1} \mathbf A^T + \mathbf Q.
\end{align*}$$


2. <u>Compute kalman gain:</u>

$$\mathbf K_k = \mathbf{\bar P}_k \mathbf H^T \left(\mathbf H \mathbf{\bar P}_k \mathbf H^T + \mathbf R\right)^{-1}.$$


3. <u>Compute the estimate (state update equation):</u>

$$\mathbf x_k = \mathbf{\bar x}_k + \mathbf K_k \left(\mathbf z_k - \mathbf H \mathbf{\bar x}_k\right).$$


4. <u>Compute the error covariance:</u>

$$\mathbf P_k = \mathbf{\bar P}_k - \mathbf K_k \mathbf H \mathbf{\bar P}_k.$$


Steps 1-4 are then repeated to recursively update with each new $\mathbf z_k$.

*Notes:*
1. *Here, the bar notations denote predicted values before measurement.*

2. *The term $(\mathbf z_k - \mathbf H \mathbf{\bar x}_k)$ in the state update equation is important, because it represents the gap between our prediction, and our measurement. Because of this importance, it is given the name "measurement residual" or "innovation".*

This can be applied to both uni-variate and multi-variate systems, and the notation is unfortunates not always consistent. Below is an explanation from Roger Labbe in his book [Kalman and Bayesian Filters in Python](https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python) (note that Labbe refers to the state transition as $\mathbf F$ rather than $\mathbf A$):

>"[the univariate and multivariate equations]... are quite similar.
>
><u>**Predict**</u>
>
>
>$
>\begin{array}{|l|l|l|}
>\hline
>\text{Univariate} & \text{Univariate} & \text{Multivariate}\\
>& \text{(Kalman form)} & \\
>\hline
>\bar \mu = \mu + \mu_{f_x} & \bar x = x + dx & \bar{\mathbf x} = \mathbf{Fx} + \mathbf{Bu}\\
>\bar\sigma^2 = \sigma_x^2 + \sigma_{f_x}^2 & \bar P = P + Q & \bar{\mathbf P} = \mathbf{FPF}^\mathsf T + \mathbf Q >\\
>\hline
>\end{array}
>$
>
>Without worrying about the specifics of the linear algebra, we can see that:
>$\mathbf x,\, \mathbf P$ are the state mean and covariance. They correspond to $x$ and $\sigma^2$.
>$\mathbf F$ is the *state transition function*. When multiplied by $\bf x$ it computes the prior.
>$\mathbf Q$ is the process covariance. It corresponds to $\sigma^2_{f_x}$.
>$\mathbf B$ and $\mathbf u$ are new to us. They let us model control inputs to the system.
>
><u>**Update**</u>
>
>
>$
>\begin{array}{|l|l|l|}
>\hline
>\text{Univariate} & \text{Univariate} & \text{Multivariate}\\
>& \text{(Kalman form)} & \\
>\hline
>& y = z - \bar x & \mathbf y = \mathbf z - \mathbf{H\bar x} \\
>& K = \frac{\bar P}{\bar P+R}&
>\mathbf K = \mathbf{\bar{P}H}^\mathsf T (\mathbf{H\bar{P}H}^\mathsf T + \mathbf R)^{-1} \\
>\mu=\frac{\bar\sigma^2\, \mu_z + \sigma_z^2 \, \bar\mu} {\bar\sigma^2 + \sigma_z^2} & x = \bar x + Ky & \mathbf x = \bar{\mathbf x} + \mathbf{Ky} \\
>\sigma^2 = \frac{\sigma_1^2\sigma_2^2}{\sigma_1^2+\sigma_2^2} & P = (1-K)\bar P & \mathbf P = (\mathbf I -\mathbf{KH})\mathbf{\bar{P}} \\
>\hline
>\end{array}
>$
>
>$\mathbf H$ is the measurement function. We haven't seen this yet in this book and I'll explain it later. If you mentally remove $\mathbf H$ from the equations, you should be able to see these equations are similar as well.
>
>$\mathbf z,\, \mathbf R$ are the measurement mean and noise covariance. They correspond to $z$ and $\sigma_z^2$ in the univariate filter (I've substituted $\mu$ with $x$ for the univariate equations to make the notation as similar as possible).
>
>$\mathbf y$ and $\mathbf K$ are the residual and Kalman gain.
>
>The details will be different than the univariate filter because these are vectors and matrices, but the concepts are exactly the same:
>-  Use a Gaussian to represent our estimate of the state and error
>-  Use a Gaussian to represent the measurement and its error
>-  Use a Gaussian to represent the process model
>-  Use the process model to predict the next state (the prior)
>-  Form an estimate part way between the measurement and the prior
>Your job as a designer will be to design the state $\left(\mathbf x, \mathbf P\right)$, the process $\left(\mathbf F, \mathbf Q\right)$, the measurement $\left(\mathbf z, \mathbf R\right)$, and the measurement function $\mathbf H$. If the system has control inputs, such as a robot, you will also design $\mathbf B$ and $\mathbf u$."

Lets try applying this to our problem.
# Designing the state: x, P

The state of the kalman filter is described by state variable $\mathbf x$ and the covariance $\mathbf P$. In this section, we will discuss how to set their initial values. After we set their initial values, our kalman filter will update them internally at each time step.

## x

As described in the *"Defining our State Variable"* section, we want $\mathbf x$ to be a be a 16x1 vector (though we could simplify it to 13x1 by removing linear acceleration). If we could set our origin at the initial position of the IMU, and we could be fairly certain that it would be stationary and aligned with $N$-$E$-$D$ axes when we start recording data, then a resonable initial state $\mathbf x_0$ might look like:

$$\begin{align*}
\mathbf p^{\text{world}}_k &= \left[0,0,0\right]^T, \\
\mathbf v^{\text{world}}_k &= \left[0,0,0\right]^T, \\
\mathbf a^{\text{world}}_k &= \left[0,0,0\right]^T, \\
\mathbf q^{\text{world}}_k &= \left[1,0,0,0\right]^T, \\
\boldsymbol\omega^{\text{world}}_k &= \left[0,0,0\right]^T, \\
\implies\mathbf x_0 &= \left[0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0\right]^T.
\end{align*}$$


## P

The state covariance $\mathbf P$ will be a 16x16 (or 13x13) matrix which represents the covariance of the state. A reasonable $\mathbf P_0$ would be:

$$\mathbf P_0 =
\begin{bmatrix}
\sigma^2_{p^{\text{N}}_0} & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 \\
0 & \sigma^2_{p^{\text{E}}_0} & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 \\
0 & 0 & \sigma^2_{p^{\text{D}}_0} & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 \\
0 & 0 & 0 & \sigma^2_{v^{\text{N}}_0} & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 \\
0 & 0 & 0 & 0 & \sigma^2_{v^{\text{E}}_0} & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 \\
0 & 0 & 0 & 0 & 0 & \sigma^2_{v^{\text{D}}_0} & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 \\
0 & 0 & 0 & 0 & 0 & 0 & \sigma^2_{a^{\text{N}}_0} & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 \\
0 & 0 & 0 & 0 & 0 & 0 & 0 & \sigma^2_{a^{\text{E}}_0} & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 \\
0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & \sigma^2_{a^{\text{D}}_0} & 0 & 0 & 0 & 0 & 0 & 0 & 0 \\
0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & \sigma^2_{q^0_0} & 0 & 0 & 0 & 0 & 0 & 0 \\
0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & \sigma^2_{q^1_0} & 0 & 0 & 0 & 0 & 0 \\
0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & \sigma^2_{q^2_0} & 0 & 0 & 0 & 0 \\
0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & \sigma^2_{q^3_0} & 0 & 0 & 0 \\
0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & \sigma^2_{\omega^{\text{N}}_0} & 0 & 0 \\
0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & \sigma^2_{\omega^{\text{E}}_0} & 0 \\
0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & \sigma^2_{\omega^{\text{D}}_0} \\
\end{bmatrix},$$

where $\sigma^2_{p^N_0}$ is the variance in the initial position in the north direction, and so on and so forth. As a general rule of thumb, it will be better to overestimate than underestimate - the filter will converge if $\mathbf P_0$ is too large, but might not if it's too small.
# Designing the process: F, Q

The process of the kalman filter is described by $\mathbf F$ (the state transition function) and $\mathbf Q$ (the process covariance).

## F

There are a few things we want $\mathbf F$ to do.

1. <b><u>Position Update</u></b>

If $dt$ is the time between measurements, then we want it to update the postion $\mathbf p = \left[p^\text{N}_k, p^\text{E}_k, p^\text{D}_k\right]^T$ in a way that satisfies

$$\mathbf p_{k+1} = \mathbf p_k + (\mathbf v_k)dt,
$$ 
where $\mathbf v = \left[v^\text{N}_k, v^\text{E}_k, v^\text{D}_k\right]^T$, and $dt$ is the time step between measurements. This expands to$$

\begin{bmatrix}p^\text N_{k+1}\\p^\text E_{k+1}\\p^\text D_{k+1}\end{bmatrix} = \begin{bmatrix}p^\text N_{k}\\p^\text E_{k}\\p^\text D_{k}\end{bmatrix} + \begin{bmatrix}v^\text N_{k}\\v^\text E_{k}\\v^\text D_{k}\end{bmatrix}dt.

$$Therefore, the top three rows of our matrix will be$$

\begin{bmatrix}
1&0&0&dt&0&0&0&0&0&0&0&0&0&0&0&0\\
0&1&0&0&dt&0&0&0&0&0&0&0&0&0&0&0\\
0&0&1&0&0&dt&0&0&0&0&0&0&0&0&0&0
\end{bmatrix}.

$$
2. <b><u>Velocity Update</u></b>

We  want it to update the velocity in way that satisfies$$

\mathbf v_{k+1} = \mathbf v_k + (\mathbf a_k - g)dt,

$$where the $-g$ term corrects for gravity. This expands to$$

\begin{align*}
\begin{bmatrix}v^\text N_{k+1}\\v^\text E_{k+1}\\v^\text D_{k+1}\end{bmatrix} &= \begin{bmatrix}v^\text N_{k}\\v^\text E_{k}\\v^\text D_{k}\end{bmatrix} + \left(\begin{bmatrix}a^\text N_{k}\\a^\text E_{k}\\a^\text D_{k}\end{bmatrix} - \begin{bmatrix}0\\0\\9.8\end{bmatrix}\right)dt, \\
&= \begin{bmatrix}v^\text N_{k}\\v^\text E_{k}\\v^\text D_{k}\end{bmatrix} + \begin{bmatrix}a^\text N_{k}\\a^\text E_{k}\\a^\text D_{k}-9.8\end{bmatrix}dt,
\end{align*}

$$Since the resulting $-dt$ term at the bottom of the vector is non linearly dependent on the components of $\mathbf x$, we can model gravity's input to the system by letting $\mathbf{Bu}$ subtract $9.8(dt)$ from the $v^\text D$ component of $\mathbf x$ in the state update equation , and the next three rows of our matrix will be$$

\begin{bmatrix}
0&0&0&1&0&0&dt&0&0&0&0&0&0&0&0&0\\
0&0&0&0&1&0&0&dt&0&0&0&0&0&0&0&0\\
0&0&0&0&0&1&0&0&dt&0&0&0&0&0&0&0
\end{bmatrix}.

$$
3. <b><u>Acceleration Update</u></b>

To keep things simple, we won't predict change to the acceleration or angular velocity and will use$$

\mathbf a_{k+1} = \mathbf a_k.

$$Therefore, next three rows of our matrix will be$$

\begin{bmatrix}
0&0&0&0&0&0&1&0&0&0&0&0&0&0&0&0\\
0&0&0&0&0&0&0&1&0&0&0&0&0&0&0&0\\
0&0&0&0&0&0&0&0&1&0&0&0&0&0&0&0
\end{bmatrix}.

$$
4. <b><u>Orientation Update</u></b>

We want it to update the orientation in a way that satisfies the quaternion update function$$

\mathbf q_{k+1} = \mathbf q_k+\frac12dt\cdot\mathbf q_k\otimes\begin{bmatrix}0 \\ \omega x_k \\ \omega y_k \\ \omega z_k\end{bmatrix}.

$$satisfy our rotation update equation.Let's start by expanding our quaternion multiplication term.

We know that the product of two quaternions$$

\mathbf q_1 = (w_1 + x_1i + y_1j + z_1k)

$$and$$

\mathbf q_2 = (w_2 + x_2i + y_2j + z_2k)

$$is calculated using the formula:$$

\begin{align*}
\mathbf q=\mathbf q_1 \otimes \mathbf q_2=&\ \ \ \left(w_1w_2 - x_1x_2 - y_1y_2 - z_1z_2\right) \\
&+ \left(w_1x_2 + x_1w_2 + y_1z_2 - z_1y_2\right)i \\
&+ \left(w_1y_2 - x_1z_2 + y_1w_2 + z_1x_2\right)j \\
&+ \left(w_1z_2 + x_1y_2 - y_1x_2 + z_1w_2\right)k.
\end{align*}

$$Substituting$$

\begin{align*}
\begin{bmatrix}w_1\\x_1\\y_1\\z_1\end{bmatrix} &= \begin{bmatrix}q^0_k\\q^1_k\\q^2_k\\q^3_k\end{bmatrix},\\
\begin{bmatrix}w_2\\x_2\\y_2\\z_2\end{bmatrix} &= \begin{bmatrix}0 \\ \omega^N_k \\ \omega^E_k \\ \omega^D_k\end{bmatrix},
\end{align*}

$$gives$$

\begin{align*}
\mathbf q_k\otimes\begin{bmatrix}0 \\ \omega^N_k \\ \omega^E_k \\ \omega^D_k\end{bmatrix} &= \begin{bmatrix}q^0_k\\q^1_k\\q^2_k\\q^3_k\end{bmatrix}\begin{bmatrix}0 \\ \omega^N_k \\ \omega^E_k \\ \omega^D_k\end{bmatrix} \\
&=\ \ \ \left(q^0_k0 - q^1_k\omega^N_k - q^2_k\omega^E_k - q^3_k\omega^D_k\right)  \\
&\ \ \ \ + \left(q^0_k\omega^N_k + q^1_k0 + q^2_k\omega^D_k - q^3_k\omega^E_k\right)i  \\
&\ \ \ \ + \left(q^0_k\omega^E_k - q^1_k\omega^D_k + q^2_k0 + q^3_k\omega^N_k\right)j  \\
&\ \ \ \ + \left(q^0_k\omega^D_k + q^1_k\omega^E_k - q^2_k\omega^N_k + q^3_k0\right)k.
\end{align*}

$$Writing this result in vector form, we have$$

\begin{bmatrix}
(q^0_k)(0) &- (q^1_k)(\omega^N_k) &- (q^2_k)(\omega^E_k) &- (q^3_k)(\omega^D_k)  \\
(q^0_k)(\omega^N_k) &+ (q^1_k)(0) &+ (q^2_k)(\omega^D_k) &- (q^3_k)(\omega^E_k)  \\
(q^0_k)(\omega^E_k) &- (q^1_k)(\omega^D_k) &+ (q^2_k)(0) &+ (q^3_k)(\omega^N_k)  \\
(q^0_k)(\omega^D_k) &+ (q^1_k)(\omega^E_k) &- (q^2_k)(\omega^N_k) &+ (q^3_k)(0) \\
\end{bmatrix}.

$$We see that each component is in the form $[a(q0_k)+b(q1_k)+c(q2_k)+d(q3_k)]$, for some constants $a$, $b$, $c$, and $d$. This is looking quite close to the form we would like for our state transition matrix! We can substite$$

\mathbf q_k\otimes\begin{bmatrix}0 \\ \omega_x \\ \omega_y \\ \omega_z\end{bmatrix} = \begin{bmatrix}
(q^0_k)(0) &- (q^1_k)(\omega^N_k) &- (q^2_k)(\omega^E_k) &- (q^3_k)(\omega^D_k)  \\
(q^0_k)(\omega^N_k) &+ (q^1_k)(0) &+ (q^2_k)(\omega^D_k) &- (q^3_k)(\omega^E_k)  \\
(q^0_k)(\omega^E_k) &- (q^1_k)(\omega^D_k) &+ (q^2_k)(0) &+ (q^3_k)(\omega^N_k)  \\
(q^0_k)(\omega^D_k) &+ (q^1_k)(\omega^E_k) &- (q^2_k)(\omega^N_k) &+ (q^3_k)(0) \\
\end{bmatrix}

$$into our rotation update equation to get$$

\mathbf q_{k+1} = \mathbf q_k+\frac12dt\cdot
\begin{bmatrix}
(q^0_k)(0) &- (q^1_k)(\omega^N_k) &- (q^2_k)(\omega^E_k) &- (q^3_k)(\omega^D_k)  \\
(q^0_k)(\omega^N_k) &+ (q^1_k)(0) &+ (q^2_k)(\omega^D_k) &- (q^3_k)(\omega^E_k)  \\
(q^0_k)(\omega^E_k) &- (q^1_k)(\omega^D_k) &+ (q^2_k)(0) &+ (q^3_k)(\omega^N_k)  \\
(q^0_k)(\omega^D_k) &+ (q^1_k)(\omega^E_k) &- (q^2_k)(\omega^N_k) &+ (q^3_k)(0) \\
\end{bmatrix}.

$$
Writing the whole right side as one vector gives$$

\mathbf q_{k+1} = \begin{bmatrix}
q^0_k + (dt/2)((q^0_k)(0) - (q^1_k)(\omega^N_k) - (q^2_k)(\omega^E_k) - (q^3_k)(\omega^D_k)) \\
q^1_k + (dt/2)((q^0_k)(\omega^N_k) + (q^1_k)(0) + (q^2_k)(\omega^D_k) - (q^3_k)(\omega^E_k)) \\
q^2_k + (dt/2)((q^0_k)(\omega^E_k) - (q^1_k)(\omega^D_k) + (q^2_k)(0) + (q^3_k)(\omega^N_k)) \\
q^3_k + (dt/2)((q^0_k)(\omega^D_k) + (q^1_k)(\omega^E_k) - (q^2_k)(\omega^N_k) + (q^3_k)(0)) \\
\end{bmatrix}.

$$Therefore, next four rows of our matrix will be$$

\begin{bmatrix}
0&0&0&0&0&0&0&0&0&1&-(dt\cdot\omega^N_k)/2&-(dt\cdot\omega^E_k)/2&-(dt\cdot\omega^D_k)/2&0&0&0\\
0&0&0&0&0&0&0&0&0&(dt\cdot\omega^N_k)/2&1&(dt\cdot\omega^D_k)/2&-(dt\cdot\omega^E_k)/2&0&0&0\\
0&0&0&0&0&0&0&0&0&(dt\cdot\omega^E_k)/2&-(dt\cdot\omega^D_k)/2&1&(dt\cdot\omega^N_k)/2&0&0&0\\
0&0&0&0&0&0&0&0&0&(dt\cdot\omega^D_k)/2&(dt\cdot\omega^E_k)/2&-(dt\cdot\omega^N_k)/2&1&0&0&0
\end{bmatrix}.

$$
5. <b><u>Angular Velocity Update</u></b>
$$

\boldsymbol\omega_{k+1} = \boldsymbol\omega_k.

$$Therefore, last three rows of our matrix will be$$

\begin{bmatrix}
0&0&0&0&0&0&0&0&0&0&0&0&0&1&0&0\\
0&0&0&0&0&0&0&0&0&0&0&0&0&0&1&0\\
0&0&0&0&0&0&0&0&0&0&0&0&0&0&0&1
\end{bmatrix}.

$$
We now have$$

\mathbf F = \begin{bmatrix}
1&0&0&dt&0&0&0&0&0&0&0&0&0&0&0&0\\
0&1&0&0&dt&0&0&0&0&0&0&0&0&0&0&0\\
0&0&1&0&0&dt&0&0&0&0&0&0&0&0&0&0\\
0&0&0&1&0&0&dt&0&0&0&0&0&0&0&0&0\\
0&0&0&0&1&0&0&dt&0&0&0&0&0&0&0&0\\
0&0&0&0&0&1&0&0&dt&0&0&0&0&0&0&0\\
0&0&0&0&0&0&1&0&0&0&0&0&0&0&0&0\\
0&0&0&0&0&0&0&1&0&0&0&0&0&0&0&0\\
0&0&0&0&0&0&0&0&1&0&0&0&0&0&0&0\\
0&0&0&0&0&0&0&0&0&1&-(dt\cdot\omega^N_k)/2&-(dt\cdot\omega^E_k)/2&-(dt\cdot\omega^D_k)/2&0&0&0\\
0&0&0&0&0&0&0&0&0&(dt\cdot\omega^N_k)/2&1&(dt\cdot\omega^D_k)/2&-(dt\cdot\omega^E_k)/2&0&0&0\\
0&0&0&0&0&0&0&0&0&(dt\cdot\omega^E_k)/2&-(dt\cdot\omega^D_k)/2&1&(dt\cdot\omega^N_k)/2&0&0&0\\
0&0&0&0&0&0&0&0&0&(dt\cdot\omega^D_k)/2&(dt\cdot\omega^E_k)/2&-(dt\cdot\omega^N_k)/2&1&0&0&0\\
0&0&0&0&0&0&0&0&0&0&0&0&0&1&0&0\\
0&0&0&0&0&0&0&0&0&0&0&0&0&0&1&0\\
0&0&0&0&0&0&0&0&0&0&0&0&0&0&0&1\\
\end{bmatrix}.

$$
*Notes:*
- *We were able to write lines 10-13 of this matrix using our  observation that each component of $\mathbf q_{k+1}$ was of the form $[a(q^0_k)+b(q^1_k)+c(q^2_k)+d(q^3_k)]$, but we could have also found the form of the $\zeta^\text{th}$ component to be $[q^\zeta_k + a(\omega^N_k)+b(\omega^E_k)+c(\omega^D_k)]$. This second form will let us write a second, equavalent matrix to help double check we have placed our values correctly.*

- *This state transition matrix assumes acceleration at time $t_{k+1}$ will approximately equal acceleration at time $t_k$, which is not necessarily true during jerky events such as heel strike.*

## Q

The process noise covariance matrix $\mathbf Q$ represents the uncertainties in the system dynamics. For our state vector, this would look like$$

\mathbf Q =
\begin{bmatrix}
\sigma^2_{p^{\text{N}}} & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 \\
0 & \sigma^2_{p^{\text{E}}} & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 \\
0 & 0 & \sigma^2_{p^{\text{D}}} & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 \\
0 & 0 & 0 & \sigma^2_{v^{\text{N}}} & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 \\
0 & 0 & 0 & 0 & \sigma^2_{v^{\text{E}}} & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 \\
0 & 0 & 0 & 0 & 0 & \sigma^2_{v^{\text{D}}} & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 \\
0 & 0 & 0 & 0 & 0 & 0 & \sigma^2_{a^{\text{N}}} & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 \\
0 & 0 & 0 & 0 & 0 & 0 & 0 & \sigma^2_{a^{\text{E}}} & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 \\
0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & \sigma^2_{a^{\text{D}}} & 0 & 0 & 0 & 0 & 0 & 0 & 0 \\
0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & \sigma^2_{q^0} & 0 & 0 & 0 & 0 & 0 & 0 \\
0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & \sigma^2_{q^1} & 0 & 0 & 0 & 0 & 0 \\
0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & \sigma^2_{q^2} & 0 & 0 & 0 & 0 \\
0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & \sigma^2_{q^3} & 0 & 0 & 0 \\
0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & \sigma^2_{\omega^{\text{N}}} & 0 & 0 \\
0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & \sigma^2_{\omega^{\text{E}}} & 0 \\
0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & \sigma^2_{\omega^{\text{D}}} \\
\end{bmatrix},

$$Where: $\sigma^2_{p^\text N}$ is the variance in the north axis position, and so on and so forth. The specific values for these variances would depend on the characteristics of the system and the expected process noise.
# Designing the Measurement: z, R

The kalman filter's measurment is described by the measurement mean $\mathbf z$, and the noise covariance $\mathbf R$.

## z

As described in the *Defining our State Variable* section, we will have$$

\mathbf z_k = \left[a^{\text{pitch}}_k, a^{\text{roll}}_k, a^{\text{yaw}}_k, \omega^{\text{pitch}}_k, \omega^{\text{roll}}_k, \omega^{\text{yaw}}_k\right]^T.

$$
## R

Since $\mathbf z$ is a 6x1 vector, $\mathbf R$ will be a 6x6 matrix representing the noise covariance of our measurements. A reasonable $\mathbf R$ would be:$$

\mathbf R =
\begin{bmatrix}
\sigma^2_{a^{\text{pitch}}} & 0 & 0 & 0 & 0 & 0 \\
0 & \sigma^2_{a^{\text{roll}}} & 0 & 0 & 0 & 0 \\
0 & 0 & \sigma^2_{a^{\text{yaw}}} & 0 & 0 & 0 \\
0 & 0 & 0 & \sigma^2_{\omega^{\text{pitch}}} & 0 & 0 \\
0 & 0 & 0 & 0 & \sigma^2_{\omega^{\text{roll}}} & 0\\
0 & 0 & 0 & 0 & 0 & \sigma^2_{\omega^{\text{yaw}}}
\end{bmatrix},

$$where $\sigma^2_{a^{\text{pitch}}}$ is the variance in the pitch acceleration measurements, and so on and so forth. If we expect the accelerometers and gyroscopes to have the same variance in all directions, we may choose to use a single value for $\sigma^2_a$ and a single value for $\sigma^2_{\omega}$.
# Designing the Measurement Function: H

Our given forms of $\mathbf x_k$ and $\mathbf z_k$ mean we'll have some 6x16 measurement function $\mathbf H$ such that$$

\mathbf y_k = \mathbf z_k - \left(\mathbf H \mathbf\cdot\mathbf x_k\right).

$$
Let's expand the right-hand side of this equation to visualize our $\mathbf H$ matrix here:$$

\mathbf y_k =
\begin{bmatrix}a^{\text{pitch}}_k \\ a^{\text{roll}}_k \\ a^{\text{yaw}}_k \\ \omega^{\text{pitch}}_k \\ \omega^{\text{roll}}_k \\ \omega^{\text{yaw}}_k\end{bmatrix}
-\begin{bmatrix}?&?&?&?&?&?&?&?&?&?&?&?&?&?&?&?\\
?&?&?&?&?&?&?&?&?&?&?&?&?&?&?&?\\
?&?&?&?&?&?&?&?&?&?&?&?&?&?&?&?\\
?&?&?&?&?&?&?&?&?&?&?&?&?&?&?&?\\
?&?&?&?&?&?&?&?&?&?&?&?&?&?&?&?\\
?&?&?&?&?&?&?&?&?&?&?&?&?&?&?&?\end{bmatrix}
\begin{bmatrix}
p^\text{N}_k \\ p^\text{E}_k \\ p^\text{D}_k \\ v^\text{N}_k \\ v^\text{E}_k \\ v^\text{D}_k \\ a^\text{N}_k \\ a^\text{E}_k \\ a^\text{D}_k \\ q^0_k \\ q^1_k \\ q^2_k \\ q^3_k \\ \omega^\text{N}_k \\ \omega^\text{E}_k \\ \omega^\text{D}_k
\end{bmatrix}.

$$We see that the first three rows will be dotted with $\mathbf x_k$ to get the local acceleration, and the next three rows will be dotted with $\mathbf x_k$ to get the local rotational velocity. Using our $\mathbf C$ matrix from *Translating Between Local and World Axes*, we have$$

\mathbf a^{\text{local}}_k = \mathbf C^T_k \cdot \mathbf a^{\text{world}}_k,

$$which expands to$$

\begin{bmatrix}a^{\text{pitch}}_k \\ a^{\text{roll}}_k \\ a^{\text{yaw}}_k\end{bmatrix} =
\begin{bmatrix}
1 - 2\big((q^2_k)^2 + (q^3_k)^2\big) & 2\big(q^1_k q^2_k - q^0_k q^3_k\big) & 2\big(q^1_k q^3_k + q^0_k q^2_k\big) \\
2\big(q^1_k q^2_k + q^0_k q^3_k\big) & 1 - 2\big((q^1_k)^2 + (q^3_k)^2\big) & 2\big(q^2_k q^3_k - q^0_k q^1_k\big) \\
2\big(q^1_k q^3_k - q^0_k q^2_k\big) & 2\big(q^2_k q^3_k + q^0_k q^1_k\big) & 1 - 2\big((q^1_k)^2 + (q^2_k)^2\big)
\end{bmatrix}^T
\begin{bmatrix}a^{\text{N}}_k \\ a^{\text{E}}_k \\ a^{\text{D}}_k\end{bmatrix}.

$$To simplify things, let's define$$

\begin{align*}
c^0_k &= 1 - 2\big((q^2_k)^2 + (q^3_k)^2\big) \\
c^1_k &= 2\big(q^1_k q^2_k - q^0_k q^3_k\big) \\
c^2_k &= 2\big(q^1_k q^3_k + q^0_k q^2_k\big) \\
c^3_k &= 2\big(q^1_k q^2_k + q^0_k q^3_k\big) \\
c^4_k &= 1 - 2\big((q^1_k)^2 + (q^3_k)^2\big) \\
c^5_k &= 2\big(q^2_k q^3_k - q^0_k q^1_k\big) \\
c^6_k &= 2\big(q^1_k q^3_k - q^0_k q^2_k\big) \\
c^7_k &= 2\big(q^2_k q^3_k + q^0_k q^1_k\big) \\
c^8_k &= 1 - 2\big((q^1_k)^2 + (q^2_k)^2\big),
\end{align*}

$$so that we can write$$

\begin{align*}
\begin{bmatrix}a^{\text{pitch}}_k \\ a^{\text{roll}}_k \\ a^{\text{yaw}}_k\end{bmatrix} &=
\begin{bmatrix}
c^0_k & c^1_k & c^2_k \\
c^3_k & c^4_k & c^5_k \\
c^6_k & c^7_k & c^8_k
\end{bmatrix}^T
\begin{bmatrix}a^{\text{N}}_k \\ a^{\text{E}}_k \\ a^{\text{D}}_k\end{bmatrix}\\
&= \begin{bmatrix}
c^0_k & c^3_k & c^6_k \\
c^1_k & c^4_k & c^7_k \\
c^2_k & c^5_k & c^8_k
\end{bmatrix}
\begin{bmatrix}a^{\text{N}}_k \\ a^{\text{E}}_k \\ a^{\text{D}}_k\end{bmatrix}
\end{align*}

$$From here, we can start to fill in the first three rows of $\mathbf H$:$$

\mathbf H = \begin{bmatrix}
0&0&0&0&0&0&c^0_k&c^3_k&c^6_k&0&0&0&0&0&0&0\\
0&0&0&0&0&0&c^1_k&c^4_k&c^7_k&0&0&0&0&0&0&0\\
0&0&0&0&0&0&c^2_k&c^5_k&c^8_k&0&0&0&0&0&0&0\\
?&?&?&?&?&?&?&?&?&?&?&?&?&?&?&?\\
?&?&?&?&?&?&?&?&?&?&?&?&?&?&?&?\\
?&?&?&?&?&?&?&?&?&?&?&?&?&?&?&?
\end{bmatrix}

$$
The bottom three rows will be quite similar.In order to find them, we will use$$

\boldsymbol\omega^{\text{local}}_k = \mathbf C^T_k \cdot \boldsymbol\omega^{\text{world}}_k,

$$Which expands to $$

\begin{bmatrix}\omega^{\text{pitch}}_k \\ \omega^{\text{roll}}_k \\ \omega^{\text{yaw}}_k\end{bmatrix} =
\begin{bmatrix}
c^0_k & c^3_k & c^6_k \\
c^1_k & c^4_k & c^7_k \\
c^2_k & c^5_k & c^8_k
\end{bmatrix}
\begin{bmatrix}\omega^{\text N}_k \\ \omega^{\text E}_k \\ \omega^{\text D}_k\end{bmatrix},

$$Meaning our matrix should look like$$

\mathbf H =
\begin{bmatrix}
0&0&0&0&0&0&c^0_k&c^1_k&c^2_k&0&0&0&0&0&0&0\\
0&0&0&0&0&0&c^3_k&c^4_k&c^5_k&0&0&0&0&0&0&0\\
0&0&0&0&0&0&c^6_k&c^7_k&c^8_k&0&0&0&0&0&0&0\\
0&0&0&0&0&0&0&0&0&0&0&0&0&c^0_k&c^3_k&c^6_k\\
0&0&0&0&0&0&0&0&0&0&0&0&0&c^1_k&c^4_k&c^7_k\\
0&0&0&0&0&0&0&0&0&0&0&0&0&c^2_k&c^5_k&c^8_k\\
\end{bmatrix}.
$$