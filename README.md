# Mobile Robot Lab 1

## Overview

## Installation
Clone the repository and install the dependencies:
```bash
git clone https://github.com/SUNTADTAWAN/lab1_ws.git
cd lab1_ws
```

## Validation
# Lab 1.1 Kinematics of Mobile Robot
## How to Run

### 1. Build the Package
Before running the simulation, ensure the package is built successfully.

```bash
colcon build
```

After the build is complete, source the setup:

```bash
source install/setup.bash
```

### 2. Launch the Simulation Environment
Start the mobile robot simulation using the following command:

```bash
ros2 launch lab1_1 spawn_robot.launch.py
```

This command spawns the robot into the simulation environment.

### 3. Run the Different Motion Models
Once the robot is spawned, run the kinematic models for comparison:

#### 3.1 Run the **Double Track Model**
```bash
ros2 run lab1_1 lab1_1_double_track.py
```

#### 3.2 Run the **No Slip Model**
```bash
ros2 run lab1_1 lab1_1_no_slip_model.py
```

Each model runs independently, and you can analyze the differences in odometry accuracy and drift by comparing them.

![1](https://github.com/user-attachments/assets/6f51433a-3348-4441-bcd4-44b10491c82b)


## Analysis of Results from the Table

### 1) Basic Model + Yaw Rate
From the image, it can be seen that **Odometry (red line) initially aligns with Ground Truth (blue line)** but as the robot turns multiple times or travels a longer distance, **the red line drifts significantly**. This occurs because the Basic Model does not account for differences between the left and right wheels.  
(If there is slip or one wheel contributes more to turning, errors occur.)  
Additionally, **Yaw Rate is highly sensitive to errors in the steer angle**, leading to miscalculated turning radius when slip or unequal friction occurs.  
This results in **the highest drift in this table** since both IK and FK do not handle slip and are also sensitive to noise.

### 2) Basic Model + Single-Track
From the image, **Odometry Path (red) still exhibits drift but is “better” than using Yaw Rate alone**.  
This model performs better than **Basic Model + Yaw Rate** because **it considers the average angle of the left and right wheels** (even though the Basic Model assumes equal angles).  
However, a drawback is that **if the front wheels experience different slip conditions**, the real wheel angles will differ, causing **modeling inaccuracies**.  
The red line still drifts but less than **Basic + Yaw Rate**  
because **Single-Track considers the dynamics of turning, particularly related to the front wheels**.  
Although drift is still present, it is improved compared to **Basic Model + Yaw Rate**.

### 3) Basic Model + Double-Track
From the image, **drift is reduced more clearly compared to using Yaw Rate or Single-Track**  
but still does not perfectly match **Ground Truth**.  
Since **Double-Track enables more detailed readings of left and right wheels**, turns are more accurate.  
However, **IK is still based on the Basic Model**, which may not command the left and right wheels realistically, leading to **slip and drift in real-world conditions**.  
While more accurate than the previous two models, some errors remain.

### 4) No Slip + Yaw Rate
From the image, **drift is still present but not as severe as in the Basic Model + Yaw Rate**.  
Since **No Slip ensures that the commanded wheel angles align with true Ackermann steering**,  
both wheels rotate their steer angles according to Ackermann geometry,  
which results in more realistic motion.  
However, since **Yaw Rate remains sensitive to noise**, even a small slip can significantly distort the calculations.  
Still, this approach is more accurate than **Basic + Yaw Rate** as its IK supports **true Ackermann geometry**.

### 5) No Slip + Single-Track
From the image, **Odometry Path (red) aligns closely with Ground Truth (blue) in many curve sections**.  
This is because **No Slip ensures more realistic left-right steer angles when turning**.  
Additionally, **the Single-Track model averages the two front wheel angles**, helping reduce noise between the front wheels.  
If slip occurs, drift can still happen, **but not as severely as in other cases**.  
This results in a **notably better outcome compared to previous models**.

### 6) No Slip + Double-Track
From the image, **Odometry Path (red) is the closest to Ground Truth (blue) among all models**.  
This is because **the No Slip Model ensures steer angles align with ideal turning**  
and **Double-Track provides detailed left/right wheel speed readings and adjusts accordingly if the two sides differ**.  
If no external factors or slip occur, **the motion and calculations are well-aligned**.  
This makes it **the most accurate model out of all six**  
since **the No Slip Model ensures correct front wheel angles, while Double-Track distinguishes rear left/right wheel speeds**.

---

## Analysis of IK and FK Models

| **Model** | **Equations Used** | **Advantages** | **Limitations** |
|-----------|------------------|--------------|-------------|
| **Basic Model + Yaw Rate** | 1) **Basic Model (IK)** using: <br> $$\omega_{\text{wheel}} = \frac{v}{r}$$ <br> and assuming equal steer angles for both front wheels. <br> 2) **Yaw Rate (FK)** using: <br> $$\omega_{\text{wheel}} = \frac{v \tan (\delta)}{L}$$ <br> then integrating yaw to update (x,y,steer angle). | - Simple computation using basic Ackermann principles for angle and yaw rate. | - Does not distinguish left/right steer angles, leading to high errors when slip occurs. <br> - Highly sensitive to noise/misreading of angles, causing drift accumulation. <br> - Turning radius calculated from $$\tan(\delta)$$ can be incorrect if the assumed angle has noise. |
| **Basic Model + Single-Track** | 1) **Basic Model (IK)** using: <br> $$\omega_{\text{wheel}} = \frac{v}{r}$$ <br> assuming equal steer angles for both front wheels. <br> 2) **Single-Track (FK)** <br> $$\delta = \frac{\delta_{\text{left}} + \delta_{\text{right}}}{2}$$ | - Single-Track averages wheel angles (although Basic Model assumes equal angles). <br> - Simpler than Double-Track but more accurate than Yaw Rate due to noise reduction. | - Does not distinguish left/right steer angles, leading to high errors when slip occurs. <br> - Turning radius from $$\tan(\delta)$$ can be inaccurate if the angle has noise. <br> - Struggles with sudden turns or significant slip differences. |
| **No Slip + Double-Track** | 1) **No Slip (IK)** using: <br> $$R = \frac{v}{\omega}$$ <br> $$\delta_{\text{left}} = \tan^{-1} \left( \frac{L}{R - \frac{T}{2}} \right)$$ <br> $$\delta_{\text{right}} = \tan^{-1} \left( \frac{L}{R + \frac{T}{2}} \right)$$ <br> 2) **Double-Track (FK)** <br> $$v = \frac{v_{\text{left}} + v_{\text{right}}}{2}$$ <br> $$\omega = \frac{v_{\text{right}} - v_{\text{left}}}{\text{track width}}$$ | - Double-Track FK separates left/right wheel speeds, enhancing accuracy. | - If slip occurs, No Slip (IK) will command incorrect values, leading to encoder mismatches with theoretical values. |

---

## Summary: Selecting the Right Model

- **Basic Model** assumes equal front wheel angles and does not consider **Ackermann geometry**, making it simple but inaccurate.
- **No Slip Model** ensures **correct Ackermann geometry** but assumes **zero slip**, making it ideal in theory but potentially inaccurate in real-world conditions.

### **Choosing the Controller for Lab 1.2**
For **Lab 1.2**, the choice of controller depends on the requirements:

- **PID Controller** is useful for simple applications.
- **Pure Pursuit Controller** works well for smooth path tracking but struggles with sharp turns.
- **MPC (Model Predictive Control)** is the most precise, as it considers future constraints.
- **Stanley Controller** balances between real-world accuracy and computational efficiency.

For practical applications, **MPC or Stanley Controller** is preferred when steering constraints must be considered, while **PID or Pure Pursuit** is suitable for simpler cases.

### References
- [Bicycle Model for Automated Driving](https://thomasfermi.github.io/Algorithms-for-Automated-Driving/Control/BicycleModel.html)
- [No Slip Condition Constraints in Kinematic Steering](https://www.mathworks.com/help/vdynblks/ref/kinematicsteering.html)
- [Forward Kinematics and Odometry Calculation](https://ieeexplore.ieee.org/document/8574906)


# Lab 1.2 Path Tracking Controller

![2](https://github.com/user-attachments/assets/53eaa1ac-9cd8-4abc-a4e8-5cfce8c33e48)
## Analysis from the Table

### PID Controller

**Basic Equation:**  
The PID Controller uses the equation  
$$ u = K_p \cdot e + K_i \cdot \int e \, dt + K_d \cdot \frac{d}{dt} e $$  

**Tunable Parameters:**  
- **Kp** affects the proportional response to yaw error.  
- **Ki** helps reduce accumulated error, but excessive values may cause overshoot.  
- **Kd** helps reduce oscillation, but improper tuning may slow response.  

**Advantages:**  
- Simple structure and adjustable for non-complex systems.  

**Disadvantages:**  
- Requires precise tuning in real environments and cannot consider future constraints.  

**Effects on Straight Paths:**  
- Proper tuning of **Kp, Ki, Kd** allows the vehicle to maintain direction effectively.  
- Lower gain values help reduce oscillation while moving straight.  

**Effects on Curves:**  
- If **Kp** is too high, overshoot may occur when turning, causing oscillation.  
- If **Kd** is too low, oscillation may not be dampened effectively.  
- If **Ki** is too high, error accumulation (**integral windup**) may slow response.  

**Summary:**  
- High **Kp** or **Kd** can cause angular velocity overshoot.  
- Low tuning may result in insufficient or slow turns.  

---

### Stanley Controller

**Basic Equation:**  
Stanley Controller calculates the steer angle from heading error and cross track error using the equation  
$$ \delta = k_{\text{heading}} \cdot (\theta_{\text{target}} - \theta_{\text{current}}) + \tan^{-1} \left( \frac{k \cdot cte_v}{v} \right) $$  

**Tunable Parameters:**  
- **stanley_k**: Primary gain for cross track error correction.  
- **adaptive_k_factor**: Adjusts gain sensitivity based on velocity.  
- **forward_speed**: The baseline speed used for computation.  

**Advantages:**  
- Simple computational structure with fast processing.  
- Adaptive gain improves response under varying conditions.  

**Disadvantages:**  
- Cannot account for future constraints.  
- Performance may decrease in highly complex paths.  

**Effects on Straight Paths:**  
- Proper **k** tuning ensures smooth tracking without oscillation.  

**Effects on Curves:**  
- If **stanley_k** or **adaptive_k_factor** is too high, **over-correction** and oscillation may occur.  
- If **forward_speed** is too high, turns may be too slow to execute.  

**Summary:**  
- **k** and velocity affect angular velocity.  
- High tuning may cause instability when turning.  
- Low tuning may lead to slow, ineffective turns.  

---

### Pure Pursuit Controller

**Basic Equation:**  
Pure Pursuit calculates curvature using  
$$ \gamma = \frac{2 \sin(\alpha)}{L_d} $$  
and the steer angle from  
$$ \delta = \tan^{-1} (\gamma \cdot L) $$  
where **𝛼** is the angle between the robot’s heading and the path at the **lookahead** point.  

**Tunable Parameters:**  
- **lookahead_distance**: Distance used to select the target.  

**Advantages:**  
- Simple structure with fast computation.  
- Works well in smoothly changing paths.  

**Disadvantages:**  
- Not suitable for sharp turns or abrupt path changes.  

**Effects on Straight Paths:**  
- Increasing **lookahead_distance** improves smoothness and stability.  

**Effects on Curves:**  
- Excessive **lookahead_distance** may cause the vehicle to cut corners.  

**Summary:**  
- **lookahead_distance** controls path-tracking accuracy.  
- Improper tuning can lead to incorrect angular velocity and inaccurate turning.  

---

### Model Predictive Control (MPC)

**Basic Equation:**  
MPC uses the **Single Track Kinematic Model** to predict future states:  
$$ x_{k+1} = x_k + v \cos (\theta_k) \Delta t $$  
$$ y_{k+1} = y_k + v \sin (\theta_k) \Delta t $$  
$$ \theta_{k+1} = \theta_k + \frac{v}{L} \tan (\delta) \Delta t $$  

**Tunable Parameters:**  
- **dt**: Time step (smaller values increase precision but add computational load).  
- **horizon**: Number of prediction steps.  

**Advantages:**  
- Considers future constraints and system limitations.  
- Adaptive to dynamic conditions.  

**Disadvantages:**  
- Computationally complex and slower than other controllers.  

**Effects on Straight Paths:**  
- More **horizon** and smaller **dt** enhance predictive accuracy.  
- Higher **max_speed** improves movement efficiency.  

**Effects on Curves:**  
- Low **horizon** or large **dt** may reduce turning accuracy.  
- Low **max_steering** may prevent sharp turns.  

**Summary:**  
- High tuning improves responsiveness but may cause oscillations.  
- Low tuning results in smoother but slower motion.  

---

## **Comparison of Linear & Angular Velocity in Each Controller**

| **Controller** | **Linear Velocity** | **Angular Velocity (or Steering)** | **Effects of High Tuning** | **Effects of Low Tuning** |
| --- | --- | --- | --- | --- |
| **MPC** | Optimization-based with constraints (min/max speed) | $ \delta $ optimized for $ \omega $ | Slow computation, sharp turns, high speed causes missed turns | Short horizon causes poor response, slow speed, turns too wide |
| **Stanley** | Often constant or reduced for sharp turns | $ \omega = \frac{v \tan(\delta)}{L} $ | High gain causes oscillation | Low gain causes delayed corrections |
| **Pure Pursuit** | Often proportional to lookahead distance | $ \delta = \arctan(\frac{2 L \sin(\alpha)}{L_d}) $ | Large lookahead cuts corners, high speed leads to missed turns | Small lookahead oscillates, low speed moves too slow |
| **PID** | Usually constant | $ \omega = K_p e + K_i \int e\,dt + K_d \frac{de}{dt} $ | High gains cause overshoot and oscillations | Low gains lead to sluggish corrections |

## **Controller Selection for Ackermann Steering Model**

### **PID Controller**
Best for simple tasks such as wheel or steering speed control. Suitable for non-complex systems without future prediction.  

### **Pure Pursuit Controller**
Good for simple path tracking with smooth turns. Not suitable for sharp or high-precision paths.  

### **Linear MPC**
Ideal for high-precision navigation, incorporating vehicle constraints, and optimizing movement.  

### **Stanley Controller**
Suitable for stable navigation across varying speeds, considering Cross Track and Heading Error.  

## **Controller Selection for Lab 1.2**
For **Lab 1.2**, **PID** or **Pure Pursuit** is easy to implement, while **MPC** or **Stanley** provides higher accuracy and constraints consideration.

# Lab 1.3 Extended Kalman Filter (EKF) Localization

## **Analysis from the Table (Annkermann Steering Model)**

### **Linear Velocity ($v_x$)**
The linear velocity ($v_x$) is estimated using **GPS and odometry measurements**.  
- GPS provides **absolute position updates** but is **low frequency and noisy**.  
- Odometry gives **continuous velocity updates** but **accumulates drift** over time.  
- **Kalman filter fuses both sources**, reducing noise while compensating for drift.  

**Effects of Process & Measurement Noise:**
- High **$Q$ (process noise)** → More uncertainty in $v_x$ → EKF trusts measurements more.  
- High **$R_{odom}$ (odometry noise)** → Less reliance on odometry velocity updates.  
- High **$R_{gps}$ (GPS noise)** → Less reliance on GPS updates, relying more on predicted velocity.  

**Drift Analysis:**
- If **only odometry** is used, drift accumulates exponentially over time.  
- If **GPS is unreliable**, EKF may introduce fluctuations in $v_x$.  
- **Combining both** maintains stable velocity estimates, reducing accumulated errors.

---

### **Angular Velocity ($\omega_z$)**
The angular velocity ($\omega_z$) is estimated from **yaw rate in odometry** and **yaw updates from GPS**.  
- Odometry measures $\omega_z$ continuously, **but noise accumulates**.  
- GPS provides yaw measurements, **but with low update rates**.  
- **EKF fuses both, maintaining accurate yaw tracking even with slip**.  

**Effects of Process & Measurement Noise:**
- High **$Q$ (process noise)** → EKF allows more uncertainty in yaw → Trusts sensor data more.  
- High **$R_{odom}$ (odometry noise)** → EKF reduces reliance on yaw rate integration.  
- High **$R_{gps}$ (GPS yaw noise)** → EKF relies more on odometry yaw updates.  

**Drift Analysis:**
- If **only odometry** is used, small errors in $\omega_z$ accumulate into large heading drift.  
- If **GPS yaw updates are frequent**, EKF corrects the drift but may oscillate.  
- **Combining both** allows stable heading estimation, even with momentary slip.  

---

## **Parameter Tuning for EKF Stability**

| **Parameter** | **Effect on Localization** | **High Value Impact** | **Low Value Impact** |
|--------------|--------------------------|-----------------------|----------------------|
| **$Q$ (Process Noise Covariance)** | Model's belief in motion model | More uncertainty → More weight on measurements | Overconfidence in motion → Poor correction from sensors |
| **$R_{odom}$ (Odometry Noise Covariance)** | Trust in odometry velocity & yaw rate | EKF ignores odometry → Higher reliance on GPS | Over-reliance on odometry → Accumulates drift quickly |
| **$R_{gps}$ (GPS Noise Covariance)** | Trust in GPS position & yaw updates | EKF ignores GPS → Heading drift from odometry errors | Over-reliance on GPS → Jumps in estimated position |

---

## **Summary: Selecting the Best EKF Configuration**
- **If GPS is reliable** → Reduce $R_{gps}$ to get better absolute positioning.  
- **If odometry is noisy** → Increase $R_{odom}$ to reduce its effect on EKF.  
- **If real-world slip exists** → Increase $Q$ slightly to allow more flexibility in prediction.  
- **If GPS updates are slow** → Keep $R_{gps}$ moderate to avoid sudden jumps in pose estimation.  

The **ideal tuning** is a balance between trusting odometry for smooth updates and using GPS to correct drift over time. 

---

## **Controller Selection for EKF-Based Localization**
### **For Simple Applications**:
- **PID Controller** (for small, controlled areas)
- **Pure Pursuit** (for smooth path tracking but less reliable in slip conditions)

### **For More Accurate Path Tracking**:
- **Stanley Controller** (if lateral error is dominant)
- **Model Predictive Control (MPC)** (if future constraints matter)

**In Lab 1.3, MPC or Stanley Controller is ideal** due to its ability to handle GPS and odometry fusion effectively. 

- **Pansiri**
- **Tadtawan Chaloempornmongkol**
