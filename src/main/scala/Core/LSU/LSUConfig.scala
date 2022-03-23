package Core.LSU

trait LSUConfig {
  def PA_WIDTH = 40

  def LSIQ_ENTRY  = 12
  def PC_LEN      = 15

  def BYTE        = "b00"
  def HALF        = "b01"
  def WORD        = "b10"
  def DWORD       = "b11"
}

object LSUConfig extends LSUConfig
