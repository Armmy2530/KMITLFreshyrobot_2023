//   // ____________________ Mk1 ____________________
//   // _____ Start the zig zag road mission _____
//   // node 1 -> 2 -> 3
//   trackline_duration(pid_forward_parameter, baseSpeed, 3430, 0, 0);
//   // node 3 -> 4 and turn left to middle road
//   trackline_L(pid1_parameter, baseSpeed, 1, 50, 80);
//   // __________ END __________

//   // ____________________ Mk2 ____________________
//   // _____ Start the zig zag road mission _____/
//   trackline_outline(pid1_parameter, baseSpeed, 50, 0);
//   tl_sensor(100);
//   trackline_outline(pid_forward_parameter, baseSpeed, 50, 0);
//   tl_sensor(70);
//   // node 3 -> 4 and turn left to middle road
//   trackline_L(pid1_parameter, baseSpeed, 1, 50, 80);
//   // __________ END __________