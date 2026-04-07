function [] = plotit(t_arr, states_arr, command_arr, JOut)

    figure(1), plot3(states_arr(:,1), states_arr(:,2), states_arr(:,3)); grid on; xlabel("x (m)"); ylabel("y (m)"); zlabel("z (m)"); title("Spacecraft Trajectory"); hold on;

    figure(2), subplot(3,3,1), plot(t_arr, states_arr(:,1)); grid on; ylabel("x (m)"); xlabel("Time (s)"); hold on; title("Position");
    figure(2), subplot(3,3,4), plot(t_arr, states_arr(:,2)); grid on; ylabel("y (m)"); xlabel("Time (s)"); hold on;
    figure(2), subplot(3,3,7), plot(t_arr, states_arr(:,3)); grid on; ylabel("z (m)"); xlabel("Time (s)"); hold on;
    figure(2), subplot(3,3,2), plot(t_arr, states_arr(:,4)); grid on; ylabel("u (m/s)"); xlabel("Time (s)"); hold on; title("Velocity");
    figure(2), subplot(3,3,5), plot(t_arr, states_arr(:,5)); grid on; ylabel("v (m/s)"); xlabel("Time (s)"); hold on;
    figure(2), subplot(3,3,8), plot(t_arr, states_arr(:,6)); grid on; ylabel("w (m/s)"); xlabel("Time (s)"); hold on;
    figure(2), subplot(3,3,3), plot(command_arr(:,1), command_arr(:,2)); grid on; ylabel("ax (m/s^2)"); xlabel("Time (s)"); hold on; title("Acceleration");
    figure(2), subplot(3,3,6), plot(command_arr(:,1), command_arr(:,3)); grid on; ylabel("ay (m/s^2)"); xlabel("Time (s)"); hold on;
    figure(2), subplot(3,3,9), plot(command_arr(:,1), command_arr(:,4)); grid on; ylabel("az (m/s^2)"); xlabel("Time (s)"); hold on;

    figure(3), plot(t_arr, states_arr(:,7)); xlabel("Time (s)"); ylabel("Mass (kg)"); title("Spacecraft Mass"); hold on; grid on;

    figure(4), plot(command_arr(:,1), command_arr(:,5)); xlabel("Time (s)"); ylabel("Thrust (N)"); title("Commanded Thrust"); hold on; grid on;
% keyboard
    figure(5), plot(JOut(:,1), JOut(:,2)); xlabel("Time (s)"); ylabel("Control Effort"); title("Control Effort"); hold on; grid on;
end