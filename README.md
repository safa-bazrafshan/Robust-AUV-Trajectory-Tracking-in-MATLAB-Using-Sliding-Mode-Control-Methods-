# Robust AUV Trajectory Tracking in MATLAB Using Sliding Mode Control Methods

This repository presents a comprehensive MATLAB-based simulation project focused on the robust trajectory tracking of Autonomous Underwater Vehicles (AUVs) using different Sliding Mode Control (SMC) strategies. The simulation includes realistic disturbances and evaluates controller performance using the Root Mean Square Error (RMSE) metric.

---

## 🎯 Overview

Autonomous Underwater Vehicles operate in uncertain and dynamic environments, where robust control is essential for accurate trajectory tracking. This project compares three control techniques:

- 🔹 Simple Sliding Mode Control (SMC)
- 🔹 Adaptive SMC (ASMC)
- 🔹 Adaptive SMC with Disturbance Observer (ASMC-DOB)

Each method is implemented in MATLAB, tested on a 3-DOF AUV model, and evaluated for tracking performance and robustness under external disturbances.

---

## 🧠 Key Features

✅ Modular and readable MATLAB code  
✅ Disturbance injection and compensation  
✅ Realistic 3-DOF AUV dynamics  
✅ Visual trajectory plots (actual vs. desired)  
✅ Performance comparison table  
✅ Final RMSE values reported  
✅ Suitable for research, education, or control projects

---

## 🗂️ Repository Structure

`plaintext
📁 auv_model/                  % AUV dynamics and disturbance model
📁 controllers/                % SMC, ASMC, ASMC-DOB implementations
📁 figures/                    % Output trajectory plots
📁 evaluation/                 % RMSE calculation script
📄 report.pdf / report.docx    % Final report (English)
📝 README.md                   % You’re here!

---

📊 Results Snapshot

Control Strategy Adaptivity Disturbance Rejection Chattering RMSE (m) Complexity

SMC ❌ ❌ High 11.22 Low
ASMC ✅ ❌ Medium 9.42 Medium
ASMC-DOB ✅ ✅ Low 10.36 High



---

📌 How to Run

1. Open MATLAB.


2. Add all folders to your MATLAB path.


3. Run any of the control scripts:

control_smc_simple.m

control_smc_adaptive.m

control_dob_asmc_step1.m



4. To evaluate performance:

Run calculate_rmse.m after the simulation.





---

📘 Requirements

MATLAB R2021 or later

No external toolboxes required

Basic familiarity with control theory is helpful



---

👩‍💻 Author

Safa Bazrafshan
M.Sc. in Electrical Engineering (Control)
ORCID: 0009-0004-4029-9550
Email: safa.bazrafshan@gmail.com

---

☕ Acknowledgments

This project was originally developed as part of an academic research initiative on robust control for marine systems. Special thanks to mentors and peers who supported the process.


---

📜 License

This project is licensed under the MIT License – feel free to use and cite it for research and academic purposes.


---

🚀 Future Work

Add Fuzzy SMC and MPC for comparison

Extend to 6-DOF dynamics

Export real-time data for HIL simulation
