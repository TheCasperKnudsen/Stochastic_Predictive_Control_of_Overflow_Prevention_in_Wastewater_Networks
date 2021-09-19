This script serves to verify the simulator, i.e., the parameters after the on/off control data 
extraction. 

1. Casadi network simulator load 
2. Run simulation given the same D_sim, same U_sim and see how the states evolve (mainly the
   the integrator states, i.e., tanks)
3. Run simulation with the Jacobians (online linearization of the Casadi nonlinear network)
   with the same D_sim, U_sim, and see how the states evolve

