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
    val bypass_data = UInt(227.W)
    val create_data = Vec(2, UInt(227.W))
    val create_div = Bool()
    val src_rdy_for_bypas = Vec(3, Bool())
  }
  val toAiq1 = new Bundle {
    val bypass_data = UInt(214.W)
    val create_data = Vec(2, UInt(214.W))
    val create_alu = Bool()
    val src_rdy_for_bypas = Vec(3, Bool())
  }
  val toAiq = new Bundle {
    val inst_src_preg = Vec(4, Vec(3, UInt(7.W)))
    val sdiq_create0_src_sel = Vec(2, Bool())
  }
  val toBiq = new Bundle {
    val bypass_data = UInt(82.W)
    val create_data = Vec(2, UInt(82.W))
    val src_rdy_for_bypas = Vec(2, Bool())
  }
  val toLsiq = new Bundle {
    val bypass_data = UInt(163.W)
    val create_data = Vec(2, UInt(163.W))
    val create_bar = Vec(2, Bool())
    val create_load = Vec(2, Bool())
    val create_no_spec = Vec(2, Bool())
    val create_store = Vec(2, Bool())

    val create0_src_rdy_for_bypas = Vec(2, Bool())
    val create0_srcvm_rdy_for_bypass = Bool()
  }
  val sdiq_create_data = Vec(2, UInt(27.W))
  val toViq0 = new Bundle{
    val bypass_data = UInt(151.W)
    val create_data = Vec(2, UInt(151.W))
    val srcv_rdy_for_bypas = Vec(3, Bool())
    val srcvm_rdy_for_bypass = Bool()
    val create_vdiv = Bool()
  }
  val toViq1 = new Bundle{
    val bypass_data = UInt(150.W)
    val create_data = Vec(2, UInt(150.W))
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

class ISStage {

}
