package Core.IDU
import chisel3._
import chisel3.util._

class ROBData extends Bundle{
  val VL_PRED         = Bool() //39
  val VL              = UInt(8.W) //38
  val VEC_DIRTY       = Bool() //30
  val VSETVLI         = Bool() //29
  val VSEW            = UInt(3.W) //28
  val VLMUL           = UInt(2.W) //25
  val NO_SPEC_MISPRED = Bool() //23
  val NO_SPEC_MISS    = Bool() //22
  val NO_SPEC_HIT     = Bool() //21
  val LOAD            = Bool() //20
  val FP_DIRTY        = Bool() //19
  val INST_NUM        = UInt(2.W) //18
  val BKPTB_INST      = Bool() //16
  val BKPTA_INST      = Bool() //15
  val BKPTB_DATA      = Bool() //14
  val BKPTA_DATA      = Bool() //13
  val STORE           = Bool() //12
  val RAS             = Bool() //11
  val PCFIFO          = Bool() //10
  val BJU             = Bool() //9
  val INTMASK         = Bool() //8
  val SPLIT           = Bool() //7
  val PC_OFFSET       = UInt(3.W) //6
  val CMPLT_CNT       = UInt(2.W) //3
  val CMPLT           = Bool() //1
  val VLD             = Bool() //0
}

class VFPU2IS extends Bundle {
  val ex1_data_vld_dupx = Bool()
  val ex1_fmla_data_vld_dupx = Bool()
  val ex1_mfvr_inst_vld_dupx = Bool()
  val ex1_preg_dupx = UInt(7.W)
  val ex1_vreg_dupx = UInt(7.W)
  val ex2_data_vld_dupx = Bool()
  val ex2_fmla_data_vld_dupx = Bool()
  val ex2_vreg_dupx = UInt(7.W)
  val ex3_data_vld_dupx = Bool()
  val ex3_vreg_dupx = UInt(7.W)
  val ex5_wb_vreg_dupx = UInt(7.W)
  val ex5_wb_vreg_vld_dupx = Bool()
}

class ISStageInput extends Bundle {
  val fromCp0 = new Bundle {
    val icgEn = Bool()
    val yyClkEn = Bool()
  }

  val pre_dispatch = new IR_preDispatch

  val ir_pipedown = new Bundle{
    val pipedown = Bool()
    val gateclk = Bool()
    val instVld = Vec(4,Bool())

    val inst_src_match = Vec(6,new ir_srcMatch)
    val instData = Vec(4, new ISData)
  }

  //aiq0 aiq1 biq lsiq sdiq viq0 viq1 vmb
  val iq_cnt_info = Vec(8, new IQCntInfo)

  val iq_create_entry = new Bundle{
    val aiq_aiq = Vec(2, Vec(2, UInt(8.W)))
    val biq_aiq = Vec(2, UInt(12.W))
    val lsiq_aiq = Vec(2, UInt(12.W))
    val sdiq_aiq = Vec(2, UInt(12.W))
    val sdiq_dp = Vec(2, UInt(12.W))//the same with sdiq_aiq
    val viq_viq = Vec(2, Vec(2, UInt(8.W)))
  }
  val lsiq_dp_create_bypass_oldest = Bool()
  val lsiq_dp_no_spec_store_vld = Bool()

  val fromRf = new Bundle{
    val preg_lch_vld_dupx = Vec(2, Bool())//pipe0 pipe1
    val dst_preg_dupx    = Vec(2, UInt(7.W))
    val vmla_lch_vld_dupx = Vec(2, Bool())//pipe6 pipe7
    val dst_vreg_dupx    = Vec(2, UInt(7.W))
  }
  val fromLSU = new Bundle {
    val ag_pipe3_load_inst_vld = Bool()
    val ag_pipe3_preg_dupx = UInt(7.W)
    val ag_pipe3_vload_inst_vld = Bool()
    val ag_pipe3_vreg_dupx = UInt(7.W)
    val dc_pipe3_load_fwd_inst_vld_dupx = Bool()
    val dc_pipe3_load_inst_vld_dupx = Bool()
    val dc_pipe3_preg_dupx = UInt(7.W)
    val dc_pipe3_vload_fwd_inst_vld = Bool()
    val dc_pipe3_vload_inst_vld_dupx = Bool()
    val dc_pipe3_vreg_dupx = UInt(7.W)
    val vmb_create0_entry = UInt(8.W)
    val vmb_create1_entry = UInt(8.W)
    val wb_pipe3_wb_preg_dupx = UInt(7.W)
    val wb_pipe3_wb_preg_vld_dupx = Bool()
    val wb_pipe3_wb_vreg_dupx = UInt(7.W)
    val wb_pipe3_wb_vreg_vld_dupx = Bool()
  }
  val fromVFPU = Vec(2, new VFPU2IS)
  val fromRTU = new Bundle {
    val flush_fe = Bool()
    val flush_is = Bool()
    val flush_stall = Bool()
    val yy_xx_flush = Bool()
    val rob_full = Bool()

    val retire_int_vld = Bool()

    val rob_inst_idd = Vec(4, UInt(7.W))
  }
  val fromIU = new Bundle{
    val div_inst_vld                 = Bool()
    val div_preg_dupx                = UInt(7.W)
    val ex2_pipe0_wb_preg_dupx       = UInt(7.W)
    val ex2_pipe0_wb_preg_vld_dupx   = Bool()
    val ex2_pipe1_mult_inst_vld_dupx = Bool()
    val ex2_pipe1_preg_dupx          = UInt(7.W)
    val ex2_pipe1_wb_preg_dupx       = UInt(7.W)
    val ex2_pipe1_wb_preg_vld_dupx   = Bool()

    val pcfifo_dis_inst_pid = Vec(4, UInt(5.W))

    val yyxxCancel = Bool()
  }
  val fromPad = new Bundle{
    val yyIcgScanEn = Bool()
  }

  val ir_type_stall_inst2_vld = Bool()
  val ir_type_stall_inst3_vld = Bool()
}

class ISStageOutput extends Bundle{
  //aiq0 aiq1 biq lsiq sdiq viq0 viq1
  val iqCreateEn = Vec(7, Vec(2, new Bundle{
    val dp_en = Bool()
    val en = Bool()
    val gateclk_en = Bool()
  }))

  val toAiq0 = new Bundle {
    val bypass_data = new AIQ0Data
    val create_data = Vec(2, new AIQ0Data)
    val create_div = Bool()
    val src_rdy_for_bypas = Vec(3, Bool())
  }
  val toAiq1 = new Bundle {
    val bypass_data = new AIQ1Data
    val create_data = Vec(2, new AIQ1Data)
    val create_alu = Bool()
    val src_rdy_for_bypas = Vec(3, Bool())
  }
  val toAiq = new Bundle {
    val inst_src_preg = Vec(4, Vec(3, UInt(7.W)))
    val sdiq_create0_src_sel = Vec(2, Bool())
  }
  val toBiq = new Bundle {
    val bypass_data = new BIQData
    val create_data = Vec(2, new BIQData)
    val src_rdy_for_bypas = Vec(2, Bool())
  }
  val toLsiq = new Bundle {
    val bypass_data = new LSIQData
    val create_data = Vec(2, new LSIQData)
    val create_bar = Vec(2, Bool())
    val create_load = Vec(2, Bool())
    val create_no_spec = Vec(2, Bool())
    val create_store = Vec(2, Bool())

    val create0_src_rdy_for_bypas = Vec(2, Bool())
    val create0_srcvm_rdy_for_bypass = Bool()
  }
  val sdiq_create_data = Vec(2, new SDIQData)
  val toViq0 = new Bundle{
    val bypass_data = new VIQData
    val create_data = Vec(2, new VIQData)
    val srcv_rdy_for_bypas = Vec(3, Bool())
    val srcvm_rdy_for_bypass = Bool()
    val create_vdiv = Bool()
  }
  val toViq1 = new Bundle{
    val bypass_data = new VIQData
    val create_data = Vec(2, new VIQData)
    val srcv_rdy_for_bypas = Vec(3, Bool())
    val srcvm_rdy_for_bypass = Bool()
  }
  val viq_inst_srcv2_vreg = Vec(4, UInt(7.W))

  val toFence = new Bundle{
    val is_pipe_empty = Bool()
  }
  val toHad = new Bundle{
    val iq_empty = Bool()
  }
  val toIU = new Bundle{
    val pcfifo_inst_num = UInt(3.W)
    val pcfifo_inst_vld = Bool()
  }
  val toLSU = new Bundle {
    val vmb_create = Vec(2, new Bundle{
      val dp_en = Bool()
      val en = Bool()
      val gateclk_en = Bool()
      val dst_ready = Bool()
      val sdiq_entry = UInt(12.W)
      val split_num = UInt(7.W)
      val unit_stride = Bool()
      val vamo = Bool()
      val vl = UInt(8.W)
      val vreg = UInt(6.W)
      val vsew = UInt(2.W)
    })
  }
  val toTop = new Bundle{
    val inst_vld = Vec(4, Bool())
    val dis_pipedown2 = Bool()
    val iq_full = Bool()
    val vmb_full = Bool()
  }
  val toIR = new Bundle{
    val dis_type_stall = Bool()
    val inst2_vld = Bool()
    val inst2_ctrl_info = new ISCtrl
    val inst3_vld = Bool()
    val inst3_ctrl_info = new ISCtrl
    val stall     = Bool()
    val inst0_sel = UInt(2.W)
    val inst_sel  = UInt(3.W)
  }
  val toRTU = new Bundle{
    val pst_dis = Vec(4, new Bundle{
      val ereg_vld = Bool()
      val freg_vld = Bool()
      val preg_vld = Bool()
      val vreg_vld = Bool()

      val dst_reg  = UInt(5.W)
      val dstv_reg = UInt(5.W)
      val ereg     = UInt(5.W)
      val ereg_iid = UInt(7.W)
      val preg     = UInt(7.W)
      val preg_iid = UInt(7.W)
      val rel_ereg = UInt(5.W)
      val rel_preg = UInt(7.W)
      val rel_vreg = UInt(6.W)
      val vreg     = UInt(6.W)
    })
    val rob_create = Vec(4, new Bundle{
      val dp_en = Bool()
      val en = Bool()
      val gateclk_en = Bool()
      val data = new ROBData
    })
  }
}

class ISStageIO extends Bundle{
  val in  = Input(new ISStageInput)
  val out = Output(new ISStageOutput)
}

class ISStage extends Module{
  val io = IO(new ISStageIO)

  //Reg
  val instVld = RegInit(VecInit(Seq.fill(4)(false.B)))
  val dis_info = RegInit(0.U.asTypeOf(new IR_preDispatch))

  val inst_src_match = RegInit(VecInit(Seq.fill(6)(0.U.asTypeOf(new ir_srcMatch))))
  //Wire
  val is_dis_stall = Wire(Bool())
  val is_dis_type_stall = Wire(Bool())

  val inst_create_data = WireInit(VecInit(Seq.fill(4)(0.U.asTypeOf(new ISData))))
  val inst_read_data = WireInit(VecInit(Seq.fill(4)(0.U.asTypeOf(new ISData))))
  //==========================================================
  //                 Instance of Gated Cell
  //==========================================================
  val inst_clk_en = io.in.ir_pipedown.gateclk || instVld.asUInt.orR


  //==========================================================
  //                IS pipeline registers
  //==========================================================
  //----------------------------------------------------------
  //            Implement of is inst valid register
  //----------------------------------------------------------
  when(io.in.fromRTU.flush_fe || io.in.fromIU.yyxxCancel){
    instVld := WireInit(VecInit(Seq.fill(4)(false.B)))
  }.elsewhen(!is_dis_stall){
    instVld := io.in.ir_pipedown.instVld
  }

  io.out.toTop.inst_vld := instVld

  io.out.toIR.inst2_vld := instVld(2)
  io.out.toIR.inst3_vld := instVld(3)

  io.out.toFence.is_pipe_empty := !instVld(0)

  //----------------------------------------------------------
  //            Implement of dispatch control register
  //----------------------------------------------------------
  when(io.in.fromRTU.flush_fe || io.in.fromIU.yyxxCancel){
    dis_info := 0.U.asTypeOf(new IR_preDispatch)
  }.elsewhen(!is_dis_stall){
    dis_info := io.in.pre_dispatch
  }
  io.out.toTop.dis_pipedown2 := dis_info.pipedown2

  //==========================================================
  //        Control signal for IS data path update
  //==========================================================
  //TODO: figure out the meaning of tpye_stall
  is_dis_type_stall :=
    dis_info.pipedown2 &&                             //if pipedown2, is inst1/2 must be valid
      (instVld(3) && io.in.ir_type_stall_inst2_vld || //  if is inst3 valid, type stall if ir inst2 valid
        !instVld(3) && io.in.ir_type_stall_inst3_vld) //  if next cycle is inst3 not valid, type stall if ir inst3 valid

  //==========================================================
  //        Control signal for IS data path update
  //==========================================================
  //1.if pipedown2, is inst1/2 must be valid, is inst0 will sel is inst2
  //2.if pipedown4, is inst0 will sel ir inst0
  io.out.toIR.inst0_sel := Cat(!dis_info.pipedown2, dis_info.pipedown2)

  //1.if pipedown2, is inst1/2 must be valid
  //  1.1 if is inst3 valid, is inst1 sel is inst3,
  //      is inst2/3 sel ir inst0/1
  //  1.2 if is inst3 not valid, is inst1/2/3 sel ir inst0/1/2
  //2.if pipedown4, is inst1/2/3 will sel ir inst1/2/3
  io.out.toIR.inst_sel := Cat(!dis_info.pipedown2, dis_info.pipedown2 && !instVld(3), dis_info.pipedown2 && instVld(3))

  //==========================================================
  //                IR/IS pipeline registers
  //==========================================================
  //----------------------------------------------------------
  //           control singals for pipeline entry
  //----------------------------------------------------------
  //TODO: if need to implement

  //----------------------------------------------------------
  //             IS pipeline registers shift MUX
  //----------------------------------------------------------
  inst_create_data(0) := MuxLookup(io.out.toIR.inst0_sel, 0.U.asTypeOf(new ISData), Seq(
    "b01".U -> inst_read_data(2),
    "b10".U -> io.in.ir_pipedown.instData(0)
  ))

  inst_create_data(1) := MuxLookup(io.out.toIR.inst_sel, 0.U.asTypeOf(new ISData), Seq(
    "b001".U -> inst_read_data(3),
    "b010".U -> io.in.ir_pipedown.instData(0),
    "b100".U -> io.in.ir_pipedown.instData(1)
  ))

  inst_create_data(2) := MuxLookup(io.out.toIR.inst_sel, 0.U.asTypeOf(new ISData), Seq(
    "b001".U -> io.in.ir_pipedown.instData(0),
    "b010".U -> io.in.ir_pipedown.instData(1),
    "b100".U -> io.in.ir_pipedown.instData(2)
  ))

  inst_create_data(3) := MuxLookup(io.out.toIR.inst_sel, 0.U.asTypeOf(new ISData), Seq(
    "b001".U -> io.in.ir_pipedown.instData(1),
    "b010".U -> io.in.ir_pipedown.instData(2),
    "b100".U -> io.in.ir_pipedown.instData(3)
  ))

  //----------------------------------------------------------
  //            pipeline entry registers instance
  //----------------------------------------------------------
  val is_dp_inst = Seq.fill(4)(Module(new ct_idu_is_pipe_entry))
  for(i <- 0 until 4){
    is_dp_inst(i).io.cp0_idu_icg_en := io.in.fromCp0.icgEn
    is_dp_inst(i).io.cp0_yy_clk_en := io.in.fromCp0.yyClkEn
    is_dp_inst(i).io.cpurst_b := !reset.asBool
    is_dp_inst(i).io.ctrl_xx_rf_pipe0_preg_lch_vld_dupx := io.in.fromRf.preg_lch_vld_dupx(0)
    is_dp_inst(i).io.ctrl_xx_rf_pipe1_preg_lch_vld_dupx := io.in.fromRf.preg_lch_vld_dupx(1)
    is_dp_inst(i).io.ctrl_xx_rf_pipe6_vmla_lch_vld_dupx := io.in.fromRf.vmla_lch_vld_dupx(0)
    is_dp_inst(i).io.ctrl_xx_rf_pipe7_vmla_lch_vld_dupx := io.in.fromRf.vmla_lch_vld_dupx(1)
    is_dp_inst(i).io.dp_xx_rf_pipe0_dst_preg_dupx := io.in.fromRf.dst_preg_dupx(0)
    is_dp_inst(i).io.dp_xx_rf_pipe1_dst_preg_dupx := io.in.fromRf.dst_preg_dupx(1)
    is_dp_inst(i).io.dp_xx_rf_pipe6_dst_vreg_dupx := io.in.fromRf.dst_vreg_dupx(0)
    is_dp_inst(i).io.dp_xx_rf_pipe7_dst_vreg_dupx := io.in.fromRf.dst_vreg_dupx(1)
    is_dp_inst(i).io.forever_cpuclk := clock.asBool
    is_dp_inst(i).io.iu_idu_div_inst_vld := io.in.fromIU.div_inst_vld
    is_dp_inst(i).io.iu_idu_div_preg_dupx := io.in.fromIU.div_preg_dupx
    is_dp_inst(i).io.iu_idu_ex2_pipe0_wb_preg_dupx := io.in.fromIU.ex2_pipe0_wb_preg_dupx
    is_dp_inst(i).io.iu_idu_ex2_pipe0_wb_preg_vld_dupx := io.in.fromIU.ex2_pipe0_wb_preg_vld_dupx
    is_dp_inst(i).io.iu_idu_ex2_pipe1_mult_inst_vld_dupx := io.in.fromIU.ex2_pipe1_mult_inst_vld_dupx
    is_dp_inst(i).io.iu_idu_ex2_pipe1_preg_dupx := io.in.fromIU.ex2_pipe1_preg_dupx
    is_dp_inst(i).io.iu_idu_ex2_pipe1_wb_preg_dupx := io.in.fromIU.ex2_pipe1_wb_preg_dupx
    is_dp_inst(i).io.iu_idu_ex2_pipe1_wb_preg_vld_dupx := io.in.fromIU.ex2_pipe1_wb_preg_vld_dupx
    is_dp_inst(i).io.lsu_idu_ag_pipe3_load_inst_vld := io.in.fromLSU
    is_dp_inst(i).io.lsu_idu_ag_pipe3_preg_dupx := io.in.fromLSU
    is_dp_inst(i).io.lsu_idu_ag_pipe3_vload_inst_vld := io.in.fromLSU
    is_dp_inst(i).io.lsu_idu_ag_pipe3_vreg_dupx := io.in.fromLSU
    is_dp_inst(i).io.lsu_idu_dc_pipe3_load_fwd_inst_vld_dupx := io.in.fromLSU
    is_dp_inst(i).io.lsu_idu_dc_pipe3_load_inst_vld_dupx := io.in.fromLSU
    is_dp_inst(i).io.lsu_idu_dc_pipe3_preg_dupx := io.in.fromLSU
    is_dp_inst(i).io.lsu_idu_dc_pipe3_vload_fwd_inst_vld := io.in.fromLSU
    is_dp_inst(i).io.lsu_idu_dc_pipe3_vload_inst_vld_dupx := io.in.fromLSU
    is_dp_inst(i).io.lsu_idu_dc_pipe3_vreg_dupx := io.in.fromLSU
    is_dp_inst(i).io.lsu_idu_wb_pipe3_wb_preg_dupx := io.in.fromLSU
    is_dp_inst(i).io.lsu_idu_wb_pipe3_wb_preg_vld_dupx := io.in.fromLSU
    is_dp_inst(i).io.lsu_idu_wb_pipe3_wb_vreg_dupx := io.in.fromLSU
    is_dp_inst(i).io.lsu_idu_wb_pipe3_wb_vreg_vld_dupx := io.in.fromLSU
    is_dp_inst(i).io.pad_yy_icg_scan_en := io.in.fromPad.yyIcgScanEn
    is_dp_inst(i).io.rtu_idu_flush_fe := io.in.fromRTU.flush_fe
    is_dp_inst(i).io.rtu_idu_flush_is := io.in.fromRTU.flush_is
    is_dp_inst(i).io.vfpu_idu_ex1_pipe6_data_vld_dupx      := io.in.fromVFPU(0).ex1_data_vld_dupx
    is_dp_inst(i).io.vfpu_idu_ex1_pipe6_fmla_data_vld_dupx := io.in.fromVFPU(0).ex1_fmla_data_vld_dupx
    is_dp_inst(i).io.vfpu_idu_ex1_pipe6_mfvr_inst_vld_dupx := io.in.fromVFPU(0).ex1_mfvr_inst_vld_dupx
    is_dp_inst(i).io.vfpu_idu_ex1_pipe6_preg_dupx          := io.in.fromVFPU(0).ex1_preg_dupx
    is_dp_inst(i).io.vfpu_idu_ex1_pipe6_vreg_dupx          := io.in.fromVFPU(0).ex1_vreg_dupx
    is_dp_inst(i).io.vfpu_idu_ex1_pipe7_data_vld_dupx      := io.in.fromVFPU(1).ex1_data_vld_dupx
    is_dp_inst(i).io.vfpu_idu_ex1_pipe7_fmla_data_vld_dupx := io.in.fromVFPU(1).ex1_fmla_data_vld_dupx
    is_dp_inst(i).io.vfpu_idu_ex1_pipe7_mfvr_inst_vld_dupx := io.in.fromVFPU(1).ex1_mfvr_inst_vld_dupx
    is_dp_inst(i).io.vfpu_idu_ex1_pipe7_preg_dupx          := io.in.fromVFPU(1).ex1_preg_dupx
    is_dp_inst(i).io.vfpu_idu_ex1_pipe7_vreg_dupx          := io.in.fromVFPU(1).ex1_vreg_dupx
    is_dp_inst(i).io.vfpu_idu_ex2_pipe6_data_vld_dupx      := io.in.fromVFPU(0).ex2_data_vld_dupx
    is_dp_inst(i).io.vfpu_idu_ex2_pipe6_fmla_data_vld_dupx := io.in.fromVFPU(0).ex2_fmla_data_vld_dupx
    is_dp_inst(i).io.vfpu_idu_ex2_pipe6_vreg_dupx          := io.in.fromVFPU(0).ex2_vreg_dupx
    is_dp_inst(i).io.vfpu_idu_ex2_pipe7_data_vld_dupx      := io.in.fromVFPU(1).ex2_data_vld_dupx
    is_dp_inst(i).io.vfpu_idu_ex2_pipe7_fmla_data_vld_dupx := io.in.fromVFPU(1).ex2_fmla_data_vld_dupx
    is_dp_inst(i).io.vfpu_idu_ex2_pipe7_vreg_dupx          := io.in.fromVFPU(1).ex2_vreg_dupx
    is_dp_inst(i).io.vfpu_idu_ex3_pipe6_data_vld_dupx      := io.in.fromVFPU(0).ex3_data_vld_dupx
    is_dp_inst(i).io.vfpu_idu_ex3_pipe6_vreg_dupx          := io.in.fromVFPU(0).ex3_vreg_dupx
    is_dp_inst(i).io.vfpu_idu_ex3_pipe7_data_vld_dupx      := io.in.fromVFPU(1).ex3_data_vld_dupx
    is_dp_inst(i).io.vfpu_idu_ex3_pipe7_vreg_dupx          := io.in.fromVFPU(1).ex3_vreg_dupx
    is_dp_inst(i).io.vfpu_idu_ex5_pipe6_wb_vreg_dupx       := io.in.fromVFPU(0).ex5_wb_vreg_dupx
    is_dp_inst(i).io.vfpu_idu_ex5_pipe6_wb_vreg_vld_dupx   := io.in.fromVFPU(0).ex5_wb_vreg_vld_dupx
    is_dp_inst(i).io.vfpu_idu_ex5_pipe7_wb_vreg_dupx       := io.in.fromVFPU(1).ex5_wb_vreg_dupx
    is_dp_inst(i).io.vfpu_idu_ex5_pipe7_wb_vreg_vld_dupx   := io.in.fromVFPU(1).ex5_wb_vreg_vld_dupx
    is_dp_inst(i).io.x_create_data       := inst_create_data(i)
    is_dp_inst(i).io.x_create_dp_en      := io.in.ir_pipedown.pipedown && !is_dis_stall
    is_dp_inst(i).io.x_create_gateclk_en := io.in.ir_pipedown.gateclk
    is_dp_inst(i).io.x_entry_vld         := instVld(i)

    inst_read_data(i) := is_dp_inst(i).io.x_read_data
  }

  //----------------------------------------------------------
  //               Output for Control Logic
  //----------------------------------------------------------
  for(i <- 0 until 4){
    io.out.toAiq.inst_src_preg(i)(0) := inst_read_data(i).src0_data.preg
    io.out.toAiq.inst_src_preg(i)(1) := inst_read_data(i).src1_data.preg
    io.out.toAiq.inst_src_preg(i)(2) := inst_read_data(i).src2_data.preg
    io.out.viq_inst_srcv2_vreg(i) := inst_read_data(i).srcv2_data.preg
  }

  //----------------------------------------------------------
  //             IS pipeline preg match create
  //----------------------------------------------------------
  val inst_create_src_match = WireInit(VecInit(Seq.fill(6)(0.U.asTypeOf(new ir_srcMatch))))
  when(io.out.toIR.inst_sel === "b001".U){
    inst_create_src_match(0) := inst_src_match(5)
    inst_create_src_match(1) := 0.U(4.W)
    inst_create_src_match(2) := 0.U(4.W)
    inst_create_src_match(3) := 0.U(4.W)
    inst_create_src_match(4) := 0.U(4.W)
    inst_create_src_match(5) := io.in.ir_pipedown.inst_src_match(0)
  }.elsewhen(io.out.toIR.inst_sel === "b010".U){
    inst_create_src_match(0) := 0.U(4.W)
    inst_create_src_match(1) := 0.U(4.W)
    inst_create_src_match(2) := 0.U(4.W)
    inst_create_src_match(3) := io.in.ir_pipedown.inst_src_match(0)
    inst_create_src_match(4) := io.in.ir_pipedown.inst_src_match(1)
    inst_create_src_match(5) := io.in.ir_pipedown.inst_src_match(3)
  }.elsewhen(io.out.toIR.inst_sel === "b100".U){
    inst_create_src_match(0) := io.in.ir_pipedown.inst_src_match(0)
    inst_create_src_match(1) := io.in.ir_pipedown.inst_src_match(1)
    inst_create_src_match(2) := io.in.ir_pipedown.inst_src_match(2)
    inst_create_src_match(3) := io.in.ir_pipedown.inst_src_match(3)
    inst_create_src_match(4) := io.in.ir_pipedown.inst_src_match(4)
    inst_create_src_match(5) := io.in.ir_pipedown.inst_src_match(5)
  }

  //----------------------------------------------------------
  //              Instance of Gated Cell
  //----------------------------------------------------------
  val dp_inst_clk_en = io.in.ir_pipedown.gateclk

  //----------------------------------------------------------
  //             IS pipeline preg match create
  //----------------------------------------------------------
  val inst_creare_dp_en = io.in.ir_pipedown.pipedown && !is_dis_stall
  when(inst_creare_dp_en){
    inst_src_match := inst_create_src_match
  }

  //==========================================================
  //          Control signal for reorder buffer create
  //==========================================================
  //-------------create enable for reorder buffer-------------
  //output create enable for control path and data path
  //create 0 always from dis_inst0_vld
  io.out.toRTU.rob_create(0).en := !is_dis_stall && dis_info.inst_vld(0)
  io.out.toRTU.rob_create(1).en := !is_dis_stall && dis_info.rob_create.en1
  io.out.toRTU.rob_create(2).en := !is_dis_stall && dis_info.rob_create.en2
  io.out.toRTU.rob_create(3).en := !is_dis_stall && dis_info.rob_create.en3

  io.out.toRTU.rob_create(0).dp_en := !io.in.fromRTU.rob_full && dis_info.inst_vld(0)
  io.out.toRTU.rob_create(1).dp_en := !io.in.fromRTU.rob_full && dis_info.rob_create.en1
  io.out.toRTU.rob_create(2).dp_en := !io.in.fromRTU.rob_full && dis_info.rob_create.en2
  io.out.toRTU.rob_create(3).dp_en := !io.in.fromRTU.rob_full && dis_info.rob_create.en3

  io.out.toRTU.rob_create(0).gateclk_en := dis_info.inst_vld(0)
  io.out.toRTU.rob_create(1).gateclk_en := dis_info.rob_create.en1
  io.out.toRTU.rob_create(2).gateclk_en := dis_info.rob_create.en2
  io.out.toRTU.rob_create(3).gateclk_en := dis_info.rob_create.en3

  //==========================================================
  //                   Create Data for ROB
  //==========================================================
  val dis_inst_pc_offset = Wire(Vec(4, UInt(3.W)))
  val dis_inst_ras = Wire(Vec(4, Bool()))
  val dis_inst_fp_dirty = Wire(Vec(4, Bool()))
  val dis_inst_vec_dirty = Wire(Vec(4, Bool()))

  for(i <- 0 until 4){
    dis_inst_pc_offset(i) := Mux(inst_read_data(i).SPLIT || inst_read_data(i).BJU, 0.U, Mux(inst_read_data(i).LENGTH, 2.U, 1.U))
    dis_inst_ras(i)       := inst_read_data(i).RTS || inst_read_data(i).PCALL
    dis_inst_fp_dirty(i)  := inst_read_data(i).dstv_vld && !inst_read_data(i).dst_vreg(6) && inst_read_data(i).DSTV_IMP ||
      inst_read_data(i).dste_vld
    dis_inst_vec_dirty(i) := inst_read_data(i).dstv_vld && inst_read_data(i).dst_vreg(6) && inst_read_data(i).DSTV_IMP ||
      inst_read_data(i).VSETVLI || inst_read_data(i).VSETVL
  }


  val rob_create_data = WireInit(VecInit(Seq.fill(4)(0.U.asTypeOf(new ROBData))))

  for(i <- 0 until 4){
    rob_create_data(i).VL_PRED := inst_read_data(i).VL_PRED
    rob_create_data(i).VL := inst_read_data(i).VL
    //rob_create_data(i).VEC_DIRTY :=
    rob_create_data(i).VSETVLI := inst_read_data(i).VSETVLI
    rob_create_data(i).VSEW := inst_read_data(i).VSEW
    rob_create_data(i).VLMUL := inst_read_data(i).VLMUL
    rob_create_data(i).NO_SPEC_MISPRED := false.B
    rob_create_data(i).NO_SPEC_MISS    := false.B
    rob_create_data(i).NO_SPEC_HIT     := false.B
    rob_create_data(i).LOAD := inst_read_data(i).LOAD
    //rob_create_data(i).FP_DIRTY :=
    //rob_create_data(i).INST_NUM :=
    //rob_create_data(i).BKPTB_INST :=
    //rob_create_data(i).BKPTA_INST :=
    rob_create_data(i).BKPTB_DATA := 0.U
    rob_create_data(i).BKPTA_DATA := 0.U
    rob_create_data(i).STORE := inst_read_data(i).STADDR
    //rob_create_data(i).RAS :=
    rob_create_data(i).PCFIFO := inst_read_data(i).PCFIFO
    rob_create_data(i).BJU := inst_read_data(i).BJU
    rob_create_data(i).INTMASK := inst_read_data(i).INTMASK
    rob_create_data(i).SPLIT := inst_read_data(i).SPLIT
    //rob_create_data(i).PC_OFFSET :=
    //rob_create_data(i).CMPLT_CNT :=
    rob_create_data(i).CMPLT := false.B
    rob_create_data(i).VLD := true.B
  }

  val dis_inst01_pc_offset  = dis_inst_pc_offset(0) + dis_inst_pc_offset(1)
  val dis_inst12_pc_offset  = dis_inst_pc_offset(1) + dis_inst_pc_offset(2)
  val dis_inst23_pc_offset  = dis_inst_pc_offset(2) + dis_inst_pc_offset(3)
  val dis_inst012_pc_offset = dis_inst_pc_offset(0) + dis_inst_pc_offset(1) + dis_inst_pc_offset(2)
  val dis_inst123_pc_offset = dis_inst_pc_offset(1) + dis_inst_pc_offset(2) + dis_inst_pc_offset(3)

  val dis_inst01_bkpta_inst  = inst_read_data(0).BKPTA_INST || inst_read_data(1).BKPTA_INST
  val dis_inst01_bkptb_inst  = inst_read_data(0).BKPTB_INST || inst_read_data(1).BKPTB_INST
  val dis_inst12_bkpta_inst  = inst_read_data(1).BKPTA_INST || inst_read_data(2).BKPTA_INST
  val dis_inst12_bkptb_inst  = inst_read_data(1).BKPTB_INST || inst_read_data(2).BKPTB_INST
  val dis_inst23_bkpta_inst  = inst_read_data(2).BKPTA_INST || inst_read_data(3).BKPTA_INST
  val dis_inst23_bkptb_inst  = inst_read_data(2).BKPTB_INST || inst_read_data(3).BKPTB_INST
  val dis_inst012_bkpta_inst = inst_read_data(0).BKPTA_INST || inst_read_data(1).BKPTA_INST || inst_read_data(2).BKPTA_INST
  val dis_inst012_bkptb_inst = inst_read_data(0).BKPTB_INST || inst_read_data(1).BKPTB_INST || inst_read_data(2).BKPTB_INST
  val dis_inst123_bkpta_inst = inst_read_data(1).BKPTA_INST || inst_read_data(2).BKPTA_INST || inst_read_data(3).BKPTA_INST
  val dis_inst123_bkptb_inst = inst_read_data(1).BKPTB_INST || inst_read_data(2).BKPTB_INST || inst_read_data(3).BKPTB_INST

  val dis_inst01_fp_dirty  = dis_inst_fp_dirty(0) || dis_inst_fp_dirty(1)
  val dis_inst12_fp_dirty  = dis_inst_fp_dirty(1) || dis_inst_fp_dirty(2)
  val dis_inst23_fp_dirty  = dis_inst_fp_dirty(2) || dis_inst_fp_dirty(3)
  val dis_inst012_fp_dirty = dis_inst_fp_dirty(0) || dis_inst_fp_dirty(1) || dis_inst_fp_dirty(2)
  val dis_inst123_fp_dirty = dis_inst_fp_dirty(1) || dis_inst_fp_dirty(2) || dis_inst_fp_dirty(3)

  val dis_inst01_vec_dirty  = dis_inst_vec_dirty(0) || dis_inst_vec_dirty(1)
  val dis_inst12_vec_dirty  = dis_inst_vec_dirty(1) || dis_inst_vec_dirty(2)
  val dis_inst23_vec_dirty  = dis_inst_vec_dirty(2) || dis_inst_vec_dirty(3)
  val dis_inst012_vec_dirty = dis_inst_vec_dirty(0) || dis_inst_vec_dirty(1) || dis_inst_vec_dirty(2)
  val dis_inst123_vec_dirty = dis_inst_vec_dirty(1) || dis_inst_vec_dirty(2) || dis_inst_vec_dirty(3)

  //----------------------------------------------------------
  //                  Create Data for Port 0
  //----------------------------------------------------------
  when(io.in.pre_dispatch.rob_create.sel0 === 0.U){//inst0
    rob_create_data(0).VEC_DIRTY  := dis_inst_vec_dirty(0)
    rob_create_data(0).FP_DIRTY   := dis_inst_fp_dirty(0)
    rob_create_data(0).INST_NUM   := 1.U
    rob_create_data(0).BKPTB_INST := inst_read_data(0).BKPTB_INST
    rob_create_data(0).BKPTA_INST := inst_read_data(0).BKPTA_INST
    rob_create_data(0).RAS        := dis_inst_ras(0)
    rob_create_data(0).PC_OFFSET  := dis_inst_pc_offset(0)
    rob_create_data(0).CMPLT_CNT  := 1.U
  }.elsewhen(io.in.pre_dispatch.rob_create.sel0 === 1.U){//inst0 and inst1
    rob_create_data(0).VEC_DIRTY  := dis_inst01_vec_dirty
    rob_create_data(0).FP_DIRTY   := dis_inst01_fp_dirty
    rob_create_data(0).INST_NUM   := 2.U
    rob_create_data(0).BKPTB_INST := dis_inst01_bkptb_inst
    rob_create_data(0).BKPTA_INST := dis_inst01_bkpta_inst
    rob_create_data(0).RAS        := dis_inst_ras(0)
    rob_create_data(0).PC_OFFSET  := dis_inst01_pc_offset
    rob_create_data(0).CMPLT_CNT  := 2.U
  }.elsewhen(io.in.pre_dispatch.rob_create.sel0 === 2.U){//inst0, inst1 and inst2
    rob_create_data(0).VEC_DIRTY  := dis_inst012_vec_dirty
    rob_create_data(0).FP_DIRTY   := dis_inst012_fp_dirty
    rob_create_data(0).INST_NUM   := 3.U
    rob_create_data(0).BKPTB_INST := dis_inst012_bkptb_inst
    rob_create_data(0).BKPTA_INST := dis_inst012_bkpta_inst
    rob_create_data(0).RAS        := dis_inst_ras(0)
    rob_create_data(0).PC_OFFSET  := dis_inst012_pc_offset
    rob_create_data(0).CMPLT_CNT  := 3.U
  }

  //----------------------------------------------------------
  //                  Create Data for Port 1
  //----------------------------------------------------------
  when(io.in.pre_dispatch.rob_create.sel1 === 0.U){//inst1
    rob_create_data(1).VEC_DIRTY  := dis_inst_vec_dirty(1)
    rob_create_data(1).FP_DIRTY   := dis_inst_fp_dirty(1)
    rob_create_data(1).INST_NUM   := 1.U
    rob_create_data(1).BKPTB_INST := inst_read_data(1).BKPTB_INST
    rob_create_data(1).BKPTA_INST := inst_read_data(1).BKPTA_INST
    rob_create_data(1).RAS        := dis_inst_ras(1)
    rob_create_data(1).PC_OFFSET  := dis_inst_pc_offset(1)
    rob_create_data(1).CMPLT_CNT  := 1.U
  }.elsewhen(io.in.pre_dispatch.rob_create.sel1 === 1.U){//inst1 and inst2
    rob_create_data(1).VEC_DIRTY  := dis_inst12_vec_dirty
    rob_create_data(1).FP_DIRTY   := dis_inst12_fp_dirty
    rob_create_data(1).INST_NUM   := 2.U
    rob_create_data(1).BKPTB_INST := dis_inst12_bkptb_inst
    rob_create_data(1).BKPTA_INST := dis_inst12_bkpta_inst
    rob_create_data(1).RAS        := dis_inst_ras(1)
    rob_create_data(1).PC_OFFSET  := dis_inst12_pc_offset
    rob_create_data(1).CMPLT_CNT  := 2.U
  }.elsewhen(io.in.pre_dispatch.rob_create.sel1 === 2.U){//inst2
    rob_create_data(1).VEC_DIRTY  := dis_inst_vec_dirty(2)
    rob_create_data(1).FP_DIRTY   := dis_inst_fp_dirty(2)
    rob_create_data(1).INST_NUM   := 1.U
    rob_create_data(1).BKPTB_INST := inst_read_data(2).BKPTB_INST
    rob_create_data(1).BKPTA_INST := inst_read_data(2).BKPTA_INST
    rob_create_data(1).RAS        := dis_inst_ras(2)
    rob_create_data(1).PC_OFFSET  := dis_inst_pc_offset(2)
    rob_create_data(1).CMPLT_CNT  := 1.U
  }.elsewhen(io.in.pre_dispatch.rob_create.sel1 === 3.U){//inst3
    rob_create_data(1).VEC_DIRTY  := dis_inst_vec_dirty(3)
    rob_create_data(1).FP_DIRTY   := dis_inst_fp_dirty(3)
    rob_create_data(1).INST_NUM   := 1.U
    rob_create_data(1).BKPTB_INST := inst_read_data(3).BKPTB_INST
    rob_create_data(1).BKPTA_INST := inst_read_data(3).BKPTA_INST
    rob_create_data(1).RAS        := dis_inst_ras(3)
    rob_create_data(1).PC_OFFSET  := dis_inst_pc_offset(3)
    rob_create_data(1).CMPLT_CNT  := 1.U
  }.elsewhen(io.in.pre_dispatch.rob_create.sel1 === 4.U){//inst1, inst2 and inst3
    rob_create_data(1).VEC_DIRTY  := dis_inst123_vec_dirty
    rob_create_data(1).FP_DIRTY   := dis_inst123_fp_dirty
    rob_create_data(1).INST_NUM   := 3.U
    rob_create_data(1).BKPTB_INST := dis_inst123_bkptb_inst
    rob_create_data(1).BKPTA_INST := dis_inst123_bkpta_inst
    rob_create_data(1).RAS        := dis_inst_ras(1)
    rob_create_data(1).PC_OFFSET  := dis_inst123_pc_offset
    rob_create_data(1).CMPLT_CNT  := 3.U
  }

  //----------------------------------------------------------
  //                  Create Data for Port 2
  //----------------------------------------------------------
  when(io.in.pre_dispatch.rob_create.sel2 === 0.U){//inst2
    rob_create_data(2).VEC_DIRTY  := dis_inst_vec_dirty(2)
    rob_create_data(2).FP_DIRTY   := dis_inst_fp_dirty(2)
    rob_create_data(2).INST_NUM   := 1.U
    rob_create_data(2).BKPTB_INST := inst_read_data(2).BKPTB_INST
    rob_create_data(2).BKPTA_INST := inst_read_data(2).BKPTA_INST
    rob_create_data(2).RAS        := dis_inst_ras(2)
    rob_create_data(2).PC_OFFSET  := dis_inst_pc_offset(2)
    rob_create_data(2).CMPLT_CNT  := 1.U
  }.elsewhen(io.in.pre_dispatch.rob_create.sel2 === 2.U){//inst2 and inst3
    rob_create_data(2).VEC_DIRTY  := dis_inst23_vec_dirty
    rob_create_data(2).FP_DIRTY   := dis_inst23_fp_dirty
    rob_create_data(2).INST_NUM   := 2.U
    rob_create_data(2).BKPTB_INST := dis_inst23_bkptb_inst
    rob_create_data(2).BKPTA_INST := dis_inst23_bkpta_inst
    rob_create_data(2).RAS        := dis_inst_ras(2)
    rob_create_data(2).PC_OFFSET  := dis_inst23_pc_offset
    rob_create_data(2).CMPLT_CNT  := 2.U
  }.elsewhen(io.in.pre_dispatch.rob_create.sel2 === 3.U){//inst3
    rob_create_data(2).VEC_DIRTY  := dis_inst_vec_dirty(3)
    rob_create_data(2).FP_DIRTY   := dis_inst_fp_dirty(3)
    rob_create_data(2).INST_NUM   := 1.U
    rob_create_data(2).BKPTB_INST := inst_read_data(3).BKPTB_INST
    rob_create_data(2).BKPTA_INST := inst_read_data(3).BKPTA_INST
    rob_create_data(2).RAS        := dis_inst_ras(3)
    rob_create_data(2).PC_OFFSET  := dis_inst_pc_offset(3)
    rob_create_data(2).CMPLT_CNT  := 1.U
  }

  //----------------------------------------------------------
  //                  Create Data for Port 3
  //----------------------------------------------------------
  //create port 3 is always from inst3
  rob_create_data(3).VEC_DIRTY  := dis_inst_vec_dirty(3)
  rob_create_data(3).FP_DIRTY   := dis_inst_fp_dirty(3)
  rob_create_data(3).INST_NUM   := 1.U
  rob_create_data(3).BKPTB_INST := inst_read_data(3).BKPTB_INST
  rob_create_data(3).BKPTA_INST := inst_read_data(3).BKPTA_INST
  rob_create_data(3).RAS        := dis_inst_ras(3)
  rob_create_data(3).PC_OFFSET  := dis_inst_pc_offset(3)
  rob_create_data(3).CMPLT_CNT  := 1.U

  //----------------------------------------------------------
  //                       Assign IID
  //----------------------------------------------------------
  val inst_iid = Wire(Vec(4, UInt(7.W)))

  inst_iid(0) := io.in.fromRTU.rob_inst_idd(0)
  inst_iid(1) := Mux(io.in.pre_dispatch.pst_create_iid_sel(0)(0), io.in.fromRTU.rob_inst_idd(0), io.in.fromRTU.rob_inst_idd(1))
  inst_iid(1) := MuxLookup(io.in.pre_dispatch.pst_create_iid_sel(1), 0.U(7.W), Seq(
    "b001".U -> io.in.fromRTU.rob_inst_idd(0),
    "b010".U -> io.in.fromRTU.rob_inst_idd(1),
    "b100".U -> io.in.fromRTU.rob_inst_idd(2)
  ))
  inst_iid(2) := MuxLookup(io.in.pre_dispatch.pst_create_iid_sel(2), 0.U(7.W), Seq(
    "b001".U -> io.in.fromRTU.rob_inst_idd(1),
    "b010".U -> io.in.fromRTU.rob_inst_idd(2),
    "b100".U -> io.in.fromRTU.rob_inst_idd(3)
  ))

  //==========================================================
  //                Control signal for PST
  //==========================================================



}
