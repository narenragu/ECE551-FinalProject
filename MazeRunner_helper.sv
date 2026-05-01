package MazeRunner_helper;

    typedef enum logic [15:0] {
        CMD_CALIBRATE = 16'h0000,
        CMD_HDG_NORTH = 16'h2000,
        CMD_HDG_SOUTH = 16'h27FF,
        CMD_HDG_EAST = 16'h2C00,
        CMD_HDG_WEST = 16'h23FF,
        CMD_MOVE_STOP_L = 16'h4002, // stop left
        CMD_MOVE_STOP_R = 16'h4001, // stop right
        CMD_SOLVE_LEFT = 16'h6001, // left affinity
        CMD_SOLVE_RIGHT = 16'h6000  // right affinity
    } maze_cmd_t;

    typedef enum logic [11:0] {
        HDG_NORTH = 12'h000,
        HDG_WEST = 12'h3FF,
        HDG_SOUTH = 12'h7FF,
        HDG_EAST = 12'hC00
    } heading_t;

endpackage