This project develops a fully validated simulation of a natural‑gas dual‑valve manifold — the system used at city gates to safely reduce transmission pressure and deliver consistent downstream flow. Physical testing of these systems is expensive and potentially hazardous, so the goal was to create a realistic model that allows safe controller design and tuning.​

I built a Simulink model that includes nonlinear valve characteristics, actuator and sensor dynamics, and fluid‑mechanics‑based pressure and flow equations. The system uses two coordinated PI controllers: one regulating manifold pressure and the other regulating downstream flow.​

A key challenge was that tuning the loops independently caused instability, so I developed a tuning strategy where the flow loop is made fast and the pressure loop slower, preventing interaction. The final model accurately reproduces open‑ and closed‑loop behavior and allows engineers to generate families of tuned responses for different performance goals.​

This simulation provides a safe, flexible platform for testing control strategies and can be extended to emergency shutdown logic, high‑pressure regulators, and other gas‑manifold applications.

​
