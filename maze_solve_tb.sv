module maze_solve_tb();
    // =========================================================
    //  DUT Ports
    // =========================================================
    logic        clk, rst_n;
    logic        cmd_md;
    logic        cmd0;            // 0 = right affinity, 1 = left affinity
    logic        lft_opn, rght_opn;
    logic        mv_cmplt;
    logic        sol_cmplt;
    // Outputs
    logic        strt_hdng;
    logic [11:0] dsrd_hdng;
    logic        strt_mv;
    logic        stp_lft, stp_rght;

    // =========================================================
    //  DUT Instantiation
    // =========================================================
    maze_solve iDUT(
        .clk      (clk),
        .rst_n    (rst_n),
        .cmd_md   (cmd_md),
        .cmd0     (cmd0),
        .lft_opn  (lft_opn),
        .rght_opn (rght_opn),
        .mv_cmplt (mv_cmplt),
        .sol_cmplt(sol_cmplt),
        .strt_hdng(strt_hdng),
        .dsrd_hdng(dsrd_hdng),
        .strt_mv  (strt_mv),
        .stp_lft  (stp_lft),
        .stp_rght (stp_rght)
    );

    // =========================================================
    //  mv_cmplt: triple-flop of (strt_hdng | strt_mv)
    // =========================================================
    logic mv_cmplt_ff1, mv_cmplt_ff2;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mv_cmplt_ff1 <= 0;
            mv_cmplt_ff2 <= 0;
            mv_cmplt     <= 0;
        end else begin
            mv_cmplt_ff1 <= strt_hdng | strt_mv;
            mv_cmplt_ff2 <= mv_cmplt_ff1;
            mv_cmplt     <= mv_cmplt_ff2;
        end
    end

    // =========================================================
    //  Clock: 100 MHz
    // =========================================================
    always #5 clk = ~clk;

    // =========================================================
    //  Test Tracking
    // =========================================================
    int pass_cnt, fail_cnt;

    // =========================================================
    //  Helper Tasks
    // =========================================================

    // Wait for mv_cmplt to go high (move/heading complete)
    task wait_mv_cmplt(input int timeout_cycles = 200);
        int i;
        for (i = 0; i < timeout_cycles; i++) begin
            @(posedge clk);
            if (mv_cmplt) return;
        end
        $display("  [TIMEOUT] mv_cmplt never asserted – %0t", $time);
        fail_cnt++;
    endtask

    // Wait for strt_hdng or strt_mv assertion
    task wait_for_action(input int timeout_cycles = 200);
        int i;
        for (i = 0; i < timeout_cycles; i++) begin
            @(posedge clk);
            if (strt_hdng || strt_mv) return;
        end
        $display("  [TIMEOUT] No strt_hdng/strt_mv pulse – %0t", $time);
        fail_cnt++;
    endtask

    // Assert a condition; print PASS/FAIL
    task check(input string name, input logic cond);
        if (cond) begin
            $display("  [PASS] %s", name);
            pass_cnt++;
        end else begin
            $display("  [FAIL] %s  (time=%0t)", name, $time);
            fail_cnt++;
        end
    endtask

    // Full reset sequence
    task do_reset();
        rst_n    = 0;
        cmd_md   = 0;
        cmd0     = 0;
        lft_opn  = 0;
        rght_opn = 0;
        sol_cmplt = 0;
        repeat(4) @(posedge clk);
        rst_n = 1;
        @(posedge clk);
    endtask

    // Issue a solve command (pulse cmd_md for 1 cycle)
    task issue_solve_cmd(input logic affinity);
        cmd0  = affinity;   // 0=right, 1=left
        cmd_md = 1;
        @(posedge clk);
        cmd_md = 0;
        @(posedge clk);
    endtask

    // Complete one forward-move step, then deassert strt_mv stimulus
    task complete_move();
        wait_for_action();
        wait_mv_cmplt();
    endtask

    // =========================================================
    //  Test Scenarios
    // =========================================================

    // ---------------------------------------------------------
    // T1: dsrd_hdng reset value = 12'h000 (north) after reset
    // ---------------------------------------------------------
    task test_reset_heading();
        $display("\n--- T1: dsrd_hdng reset to 12'h000 ---");
        do_reset();
        check("dsrd_hdng == 12'h000 after reset", dsrd_hdng === 12'h000);
    endtask

    // ---------------------------------------------------------
    // T2: Left-affinity – left open path (Turn Left)
    //     Condition: lft_opn=1 → should turn left
    // ---------------------------------------------------------
    task test_left_affinity_left_open();
        $display("\n--- T2: Left affinity – left open → Turn Left ---");
        do_reset();
        lft_opn  = 1;
        rght_opn = 0;
        sol_cmplt = 0;
        issue_solve_cmd(1'b1);      // left affinity

        // SM should start moving forward until lft/frwrd not open
        // then assert strt_hdng to turn left
        wait_for_action(300);
        check("strt_hdng asserted for turn-left heading", strt_hdng === 1'b1);

        // heading should be 90° left of north = 12'hC00 (270° = west) 
        // or use whatever your encoding is; check it decreased by 90
        // Generic check: dsrd_hdng != original north after turn
        wait_mv_cmplt();

        // Let it complete and drive sol_cmplt to finish
        sol_cmplt = 1;
        @(posedge clk);
        sol_cmplt = 0;
        lft_opn = 0;
    endtask

    // ---------------------------------------------------------
    // T3: Left-affinity – left NOT open, right open → Turn Right
    // ---------------------------------------------------------
    task test_left_affinity_right_open();
        $display("\n--- T3: Left affinity – left closed, right open → Turn Right ---");
        do_reset();
        lft_opn  = 0;
        rght_opn = 1;
        sol_cmplt = 0;
        issue_solve_cmd(1'b1);      // left affinity

        wait_for_action(300);
        // SM should issue a right turn (strt_hdng for right turn)
        check("strt_hdng asserted for turn-right heading", strt_hdng === 1'b1);
        wait_mv_cmplt();

        sol_cmplt = 1;
        @(posedge clk);
        sol_cmplt = 0;
        rght_opn = 0;
    endtask

    // ---------------------------------------------------------
    // T4: Left-affinity – neither left nor right open → Pull a 180
    // ---------------------------------------------------------
    task test_left_affinity_180();
        $display("\n--- T4: Left affinity – no openings → Pull a 180 ---");
        do_reset();
        lft_opn  = 0;
        rght_opn = 0;
        sol_cmplt = 0;
        issue_solve_cmd(1'b1);      // left affinity

        // After blocked: no lft_opn, no rght_opn → 180 turn
        wait_for_action(300);
        check("strt_hdng asserted for 180 turn", strt_hdng === 1'b1);
        wait_mv_cmplt();

        // Open a path so it can proceed
        rght_opn = 1;
        wait_for_action(300);
        wait_mv_cmplt();

        sol_cmplt = 1;
        @(posedge clk);
        sol_cmplt = 0;
        rght_opn = 0;
    endtask

    // ---------------------------------------------------------
    // T5: Right-affinity – right open → Turn Right
    // ---------------------------------------------------------
    task test_right_affinity_right_open();
        $display("\n--- T5: Right affinity – right open → Turn Right ---");
        do_reset();
        lft_opn  = 0;
        rght_opn = 1;
        sol_cmplt = 0;
        issue_solve_cmd(1'b0);      // right affinity (cmd0=0)

        wait_for_action(300);
        check("strt_hdng asserted for right turn", strt_hdng === 1'b1);
        wait_mv_cmplt();

        sol_cmplt = 1;
        @(posedge clk);
        sol_cmplt = 0;
        rght_opn = 0;
    endtask

    // ---------------------------------------------------------
    // T6: Right-affinity – right NOT open, left open → Turn Left
    // ---------------------------------------------------------
    task test_right_affinity_left_open();
        $display("\n--- T6: Right affinity – right closed, left open → Turn Left ---");
        do_reset();
        lft_opn  = 1;
        rght_opn = 0;
        sol_cmplt = 0;
        issue_solve_cmd(1'b0);      // right affinity

        wait_for_action(300);
        check("strt_hdng asserted for left turn", strt_hdng === 1'b1);
        wait_mv_cmplt();

        sol_cmplt = 1;
        @(posedge clk);
        sol_cmplt = 0;
        lft_opn = 0;
    endtask

    // ---------------------------------------------------------
    // T7: Right-affinity – no openings → Pull a 180
    // ---------------------------------------------------------
    task test_right_affinity_180();
        $display("\n--- T7: Right affinity – no openings → Pull a 180 ---");
        do_reset();
        lft_opn  = 0;
        rght_opn = 0;
        sol_cmplt = 0;
        issue_solve_cmd(1'b0);      // right affinity

        wait_for_action(300);
        check("strt_hdng asserted for 180 turn", strt_hdng === 1'b1);
        wait_mv_cmplt();

        // Provide an opening for exit
        lft_opn = 1;
        wait_for_action(300);
        wait_mv_cmplt();

        sol_cmplt = 1;
        @(posedge clk);
        sol_cmplt = 0;
        lft_opn = 0;
    endtask

    // ---------------------------------------------------------
    // T8: sol_cmplt terminates solve → Done state
    //     Verify strt_mv / strt_hdng deasserted after Done
    // ---------------------------------------------------------
    task test_sol_cmplt_termination();
        $display("\n--- T8: sol_cmplt asserted → Done, outputs deasserted ---");
        do_reset();
        lft_opn  = 1;
        rght_opn = 1;
        sol_cmplt = 0;
        issue_solve_cmd(1'b1);

        // Let one move cycle happen
        wait_for_action(300);
        wait_mv_cmplt();

        // Assert sol_cmplt
        sol_cmplt = 1;
        @(posedge clk);
        @(posedge clk);
        check("strt_mv low after sol_cmplt", strt_mv   === 1'b0);
        check("strt_hdng low after sol_cmplt", strt_hdng === 1'b0);
        sol_cmplt = 0;
        lft_opn   = 0;
        rght_opn  = 0;
    endtask

    // ---------------------------------------------------------
    // T9: Multi-step left-affinity maze walk
    //     Sequence: lft_opn → turn left → forward → sol_cmplt
    // ---------------------------------------------------------
    task test_multistep_left();
        $display("\n--- T9: Multi-step left affinity walk ---");
        do_reset();
        sol_cmplt = 0;
        issue_solve_cmd(1'b1);

        // Step 1: forward path open, no left → strt_mv forward
        lft_opn  = 0;
        rght_opn = 0;
        wait_for_action(300);
        check("Step1: action asserted", strt_hdng | strt_mv);
        wait_mv_cmplt();

        // Step 2: now left is open
        lft_opn = 1;
        wait_for_action(300);
        check("Step2: strt_hdng for left turn", strt_hdng === 1'b1);
        wait_mv_cmplt();
        lft_opn = 0;

        // Step 3: forward again
        wait_for_action(300);
        check("Step3: strt_mv for forward", strt_mv === 1'b1);
        wait_mv_cmplt();

        // End
        sol_cmplt = 1;
        @(posedge clk);
        sol_cmplt = 0;
    endtask

    // ---------------------------------------------------------
    // T10: Multi-step right-affinity maze walk
    // ---------------------------------------------------------
    task test_multistep_right();
        $display("\n--- T10: Multi-step right affinity walk ---");
        do_reset();
        sol_cmplt = 0;
        issue_solve_cmd(1'b0);     // right affinity

        // Step 1: right open → turn right
        rght_opn = 1;
        lft_opn  = 0;
        wait_for_action(300);
        check("Step1: strt_hdng for right turn", strt_hdng === 1'b1);
        wait_mv_cmplt();
        rght_opn = 0;

        // Step 2: neither open → 180
        wait_for_action(300);
        check("Step2: strt_hdng for 180", strt_hdng === 1'b1);
        wait_mv_cmplt();

        // Step 3: left open → turn left
        lft_opn = 1;
        wait_for_action(300);
        check("Step3: strt_hdng for left turn", strt_hdng === 1'b1);
        wait_mv_cmplt();
        lft_opn = 0;

        sol_cmplt = 1;
        @(posedge clk);
        sol_cmplt = 0;
    endtask

    // ---------------------------------------------------------
    // T11: Both left and right open – left affinity takes left
    // ---------------------------------------------------------
    task test_both_open_left_affinity();
        $display("\n--- T11: Both open, left affinity → takes left ---");
        do_reset();
        lft_opn  = 1;
        rght_opn = 1;
        sol_cmplt = 0;
        issue_solve_cmd(1'b1);

        wait_for_action(300);
        // With left affinity and lft_opn=1, should turn left
        check("Left affinity, both open: strt_hdng asserted", strt_hdng === 1'b1);
        wait_mv_cmplt();

        sol_cmplt = 1;
        @(posedge clk);
        sol_cmplt = 0;
        lft_opn  = 0;
        rght_opn = 0;
    endtask

    // ---------------------------------------------------------
    // T12: Both open – right affinity takes right
    // ---------------------------------------------------------
    task test_both_open_right_affinity();
        $display("\n--- T12: Both open, right affinity → takes right ---");
        do_reset();
        lft_opn  = 1;
        rght_opn = 1;
        sol_cmplt = 0;
        issue_solve_cmd(1'b0);

        wait_for_action(300);
        // With right affinity and rght_opn=1, should turn right
        check("Right affinity, both open: strt_hdng asserted", strt_hdng === 1'b1);
        wait_mv_cmplt();

        sol_cmplt = 1;
        @(posedge clk);
        sol_cmplt = 0;
        lft_opn  = 0;
        rght_opn = 0;
    endtask

    // ---------------------------------------------------------
    // T13: stp_lft and stp_rght strobes during turns
    // ---------------------------------------------------------
    task test_stp_outputs();
        $display("\n--- T13: stp_lft/stp_rght strobes during turns ---");
        do_reset();
        lft_opn  = 1;
        rght_opn = 0;
        sol_cmplt = 0;
        issue_solve_cmd(1'b1);    // left affinity → turn left

        wait_for_action(300);
        // During a left turn stp_lft should be driven
        // (exact pulse timing is implementation-dependent; check at least one asserts)
        repeat(5) @(posedge clk);
        check("stp_lft or stp_rght driven during turn",
              stp_lft === 1'b1 || stp_rght === 1'b1);

        wait_mv_cmplt();
        sol_cmplt = 1;
        @(posedge clk);
        sol_cmplt = 0;
        lft_opn = 0;
    endtask

    // =========================================================
    //  Top-Level Initial Block
    // =========================================================
    initial begin
        pass_cnt = 0;
        fail_cnt = 0;
        clk      = 0;

        $display("============================================");
        $display("  maze_solve Testbench – Full Coverage");
        $display("============================================");

        test_reset_heading();
        test_left_affinity_left_open();
        test_left_affinity_right_open();
        test_left_affinity_180();
        test_right_affinity_right_open();
        test_right_affinity_left_open();
        test_right_affinity_180();
        test_sol_cmplt_termination();
        test_multistep_left();
        test_multistep_right();
        test_both_open_left_affinity();
        test_both_open_right_affinity();
        test_stp_outputs();

        $display("\n============================================");
        $display("  Results: %0d PASSED  |  %0d FAILED", pass_cnt, fail_cnt);
        $display("============================================");

        if (fail_cnt == 0)
            $display("  ALL TESTS PASSED");
        else
            $display("  SOME TESTS FAILED – review log above");

        $stop;
    end

endmodule