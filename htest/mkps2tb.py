"""ps2 的行为测试台：主机对着一个键盘模型收扫描码、发命令。

协议照 Chapweske《The PS/2 Mouse/Keyboard Protocol》（本地 ~/download/refs/ps2/）。`tick` 写 0，
一拍就是一微秒。键盘模型按 80 微秒一位出时钟（12.5 kHz，在 10 - 16.7 kHz 之内）：
  · 设备发往主机：时钟高的时候换数据，主机在下降沿读。帧是起始 0、8 位低位先出、奇校验、停止 1
  · 主机发往设备：主机拉低时钟抑制，再拉低数据、放开时钟，这就是请求发送；模型量主机抑制了
    多久，然后出时钟，在上升沿读 8 位数据、校验、停止，第 11 个时钟拉低数据作应答
  · 模型可以不应答：第 11 个时钟不拉低数据，主机要报发送失败
  · 模型可以装死：看到请求发送也不出时钟，主机要在 15 毫秒上报错
  · 模型可以发到一半就停：主机 2 毫秒后丢掉半截帧
`tick` 复位是 99，节拍计数一上电就在走，序列开头写 0 时计数已经越过新值，这就是运行中把上限改小。

认矩阵：`tx` 关着时不测主机发送与超时。
"""
import json
import pathlib
import sys

out = pathlib.Path(sys.argv[1] if len(sys.argv) > 1 else ".")
out.mkdir(parents=True, exist_ok=True)
cfg = json.loads(sys.argv[2]) if len(sys.argv) > 2 else {}
label = cfg.get("label", "")
knobs = cfg.get("knobs", {})
tx_on = bool(knobs.get("tx", True))

CTRL, TICK, RXD, TXD, STATUS = 0x00, 0x04, 0x08, 0x0C, 0x10

tx_part = f"""
    // 主机发 0xFF（复位命令）：模型收到的就是它，校验与停止位都对；主机抑制时钟不少于 100 微秒
    wr(8'h{TXD:02X}, 32'hFF);
    waitFlag(4);
    action
      Bool wrong = False;
      if (devGot[1] != 8'hFF) begin $display("FAIL the device received %02h, want ff", devGot[1]); wrong = True; end
      if (!devParOk[1]) begin $display("FAIL the host sent a byte with even parity"); wrong = True; end
      if (!devStopOk[1]) begin $display("FAIL the host did not release data for the stop bit"); wrong = True; end
      if (inhibitRun[1] < 100) begin $display("FAIL the host inhibited the clock for %0d microseconds, want at least 100", inhibitRun[1]); wrong = True; end
      if (wrong) bad <= True;
    endaction
    rd(8'h{STATUS:02X});
    action if (rdR[3] != 0) begin $display("FAIL txerr is set after an acknowledged send"); bad <= True; end endaction
    wr(8'h{STATUS:02X}, 32'h0F);

    // 设备回 0xFA（应答）
    send(8'hFA, False);
    waitFlag(1);
    rd(8'h{RXD:02X});
    action if (rdR[7:0] != 8'hFA) begin $display("FAIL the reply reads %02h, want fa", rdR[7:0]); bad <= True; end endaction
    wr(8'h{STATUS:02X}, 32'h0F);

    // 设备收完不应答：第 11 个时钟数据是高的，主机要报发送失败而不是发送完成
    nack <= True;
    wr(8'h{TXD:02X}, 32'hF4);
    waitFlag(12);
    action if (lastSt[3:2] != 2'b10) begin $display("FAIL a send the device did not acknowledge reads txdone %0d txerr %0d, want 0 and 1", lastSt[2], lastSt[3]); bad <= True; end endaction
    nack <= False;
    wr(8'h{STATUS:02X}, 32'h0F);

    // 设备装死：主机 15 毫秒上报发送失败，而且放开两根线
    mute <= True;
    wr(8'h{TXD:02X}, 32'hED);
    waitFlag(8);
    action if (d.pins.clk_pull == 1 || d.pins.data_pull == 1) begin $display("FAIL the host still pulls a line after a failed send"); bad <= True; end endaction
    mute <= False;
    wr(8'h{STATUS:02X}, 32'h0F);""" if tx_on else ""

verdict = ("a scan code frame is received, a frame cut off halfway is dropped after 2 ms with tick lowered while counting, "
           "a frame with even parity sets rxerr, and rxv drives irq through ien"
           + ("; a command goes out after at least 100 microseconds of clock inhibit with odd parity and a stop bit, "
              "the device reply is received, a send without acknowledge sets txerr, "
              "and a device that never clocks sets txerr and frees the lines" if tx_on else ""))

TEMPLATE = r'''package Ps2@L@Tb;

// 由 htest/mkps2tb.py 生成，勿手改。这一点：tx=@TXON@

import StmtFSM::*;
import ConfigReg::*;
import RegIf::*;
import Ps2::*;

typedef enum { DIdle, DWatch, DTx, DRx } DevSt deriving (Bits, Eq);

(* synthesize *)
module mkPs2@L@Tb(Empty);
  Ps2Ifc#(8, 32) d <- mkPs2(Ps2Cfg { tx: @TX@ });

  // ---- 键盘模型 ----
  Reg#(DevSt)     ds       <- mkReg(DIdle);
  Reg#(UInt#(8))  dt       <- mkReg(0);
  Reg#(UInt#(5))  dk       <- mkReg(0);
  Reg#(Bit#(11))  frame    <- mkReg(0);
  Reg#(UInt#(5))  last     <- mkReg(10);
  Reg#(Bit#(10))  rxBits   <- mkReg(0);
  Reg#(Bit#(1))   dClk     <- mkReg(0);   // 1 是模型在拉低
  Reg#(Bit#(1))   dDat     <- mkReg(0);
  Reg#(UInt#(16)) hostLow  <- mkReg(0);
  Reg#(Bool)      mute     <- mkReg(False);
  Reg#(Bool)      nack     <- mkReg(False);
  Reg#(Bool)      sendReq[2]    <- mkCReg(2, False);
  Reg#(Bit#(11))  sendFrame[2]  <- mkCReg(2, 0);
  // 帧长在模型开始发这一帧时才取：主机一报收到，序列就往下走，模型这时还在发停止位的后半拍
  Reg#(UInt#(5))  sendLast[2]   <- mkCReg(2, 10);
  Reg#(UInt#(16)) inhibitRun[2] <- mkCReg(2, 0);
  Reg#(Bit#(8))   devGot[2]     <- mkCReg(2, 0);
  Reg#(Bool)      devParOk[2]   <- mkCReg(2, False);
  Reg#(Bool)      devStopOk[2]  <- mkCReg(2, False);

  Bit#(1) busClk  = (d.pins.clk_pull == 1 || dClk == 1) ? 0 : 1;
  Bit#(1) busData = (d.pins.data_pull == 1 || dDat == 1) ? 0 : 1;

  rule bus;
    d.pins.clk_in(busClk);
    d.pins.data_in(busData);
  endrule

  rule dev;
    Bit#(1) hc = d.pins.clk_pull;
    Bit#(1) hd = d.pins.data_pull;
    hostLow <= (hc == 1) ? hostLow + 1 : 0;
    Bit#(1) nc = dClk;
    Bit#(1) nd = dDat;
    case (ds)
      DIdle: begin
        nc = 0; nd = 0;
        if (hc == 1) ds <= DWatch;
        else if (sendReq[0]) begin
          frame <= sendFrame[0]; last <= sendLast[0]; sendReq[0] <= False;
          ds <= DTx; dt <= 0; dk <= 0;
        end
      end
      // 主机拉着时钟；它放开时钟时数据是低的，就是请求发送
      DWatch: begin
        if (hc == 0) begin
          inhibitRun[0] <= hostLow;
          if (hd == 1 && !mute) begin ds <= DRx; dt <= 0; dk <= 0; end
          else ds <= DIdle;
        end
      end
      // 设备发往主机：每位 80 微秒，第 0 拍换数据，0 - 39 时钟高，40 - 79 时钟低（主机在下降沿读）
      DTx: begin
        nd = (frame[dk] == 0) ? 1 : 0;
        nc = (dt >= 40) ? 1 : 0;
        if (dt == 79) begin
          dt <= 0;
          if (dk == last) begin ds <= DIdle; nc = 0; nd = 0; end
          else dk <= dk + 1;
        end else dt <= dt + 1;
      end
      // 主机发往设备：每个时钟 80 微秒，0 - 39 低、40 - 79 高，第 40 拍（上升沿）读数据；
      // 读完 10 位（8 位数据、校验、停止），第 11 个时钟整个拉低数据作应答
      DRx: begin
        nc = (dt < 40) ? 1 : 0;
        if (dt == 40 && dk < 10) rxBits <= {busData, rxBits[9:1]};
        nd = (dk == 10 && !nack) ? 1 : 0;
        if (dt == 79) begin
          dt <= 0;
          if (dk == 10) begin
            devGot[0] <= rxBits[7:0];
            devParOk[0] <= (^rxBits[8:0]) == 1;
            devStopOk[0] <= rxBits[9] == 1;
            ds <= DIdle; nc = 0; nd = 0;
          end else dk <= dk + 1;
        end else dt <= dt + 1;
      end
    endcase
    dClk <= nc;
    dDat <= nd;
  endrule

  // ---- 命令序列 ----
  Reg#(Bool)      bad   <- mkReg(False);
  Reg#(Bit#(32))  rdR   <- mkReg(0);
  Reg#(Bool)      waitR <- mkReg(False);
  Reg#(UInt#(32)) cyc   <- mkConfigReg(0);

  function Action wr(Bit#(8) a, Bit#(32) v) = action
    let _ <- d.regs.access(RegReq { addr: a, write: True, wdata: v, wstrb: 4'hF });
  endaction;

  function Action rd(Bit#(8) a) = action
    let x <- d.regs.access(RegReq { addr: a, write: False, wdata: 0, wstrb: 4'hF });
    rdR <= x.rdata;
  endaction;

  // 起始 0、8 位低位先出、奇校验、停止 1；坏帧把校验位翻过来
  // par 是 BSV 的关键字，变量名不能叫它
  function Action send(Bit#(8) b, Bool badParity) = action
    Bit#(1) odd = ~(^b);
    sendFrame[1] <= {1'b1, badParity ? ~odd : odd, b, 1'b0};
    sendReq[1] <= True;
    sendLast[1] <= 10;
  endaction;

  // 发完起始位和前 n - 1 位就停，像设备发到一半被拔掉
  function Action sendCut(Bit#(8) b, UInt#(5) n) = action
    sendFrame[1] <= {2'b11, b, 1'b0};
    sendReq[1] <= True;
    sendLast[1] <= n - 1;
  endaction;

  // 等状态位有个上限：30 毫秒（一拍一微秒）还没等到就报出是哪一位，
  // 不然桩实现上整个序列卡死、只剩一句 TIMEOUT，看不出红在哪条判据上
  Reg#(UInt#(32)) waited <- mkReg(0);
  Reg#(Bit#(32))  lastSt <- mkReg(0);

  function Stmt waitFlag(Bit#(32) m) = seq
    waitR <= True;
    waited <= 0;
    while (waitR) action
      let x <- d.regs.access(RegReq { addr: 8'h@STATUS@, write: False, wdata: 0, wstrb: 4'hF });
      lastSt <= x.rdata;
      waited <= waited + 1;
      waitR <= (x.rdata & m) == 0 && waited < 30000;
    endaction
    action
      if ((lastSt & m) == 0) begin
        $display("FAIL waited 30 ms for status mask %02h and it never came up (status %02h, host pulls clock %0d data %0d, device state %0d)",
                 m, lastSt, d.pins.clk_pull, d.pins.data_pull, pack(ds));
        bad <= True;
      end
    endaction
  endseq;

  Stmt test = seq
    wr(8'h@TICK@, 0);
    wr(8'h@CTRL@, 1);

    // 设备发扫描码 0x1C（A 键按下）
    send(8'h1C, False);
    waitFlag(1);
    rd(8'h@RXD@);
    action if (rdR[7:0] != 8'h1C) begin $display("FAIL the scan code reads %02h, want 1c", rdR[7:0]); bad <= True; end endaction
    rd(8'h@STATUS@);
    action if (rdR[1] != 0) begin $display("FAIL rxerr is set after a good frame"); bad <= True; end endaction
    action if (d.irq) begin $display("FAIL irq rises with rxv set but ien off"); bad <= True; end endaction
    wr(8'h@CTRL@, 3);
    action if (!d.irq) begin $display("FAIL irq stays low with rxv set and ien on"); bad <= True; end endaction
    wr(8'h@CTRL@, 1);
    wr(8'h@STATUS@, 32'h0F);

    // 设备发到第 4 位就停：主机 2 毫秒等不到时钟要丢掉这半截，下一帧才对得上。
    // 开头写 tick 时节拍计数已经走过 0，比「等于」的写法要等 16 位回绕才走下一微秒，
    // 这半截就丢不掉，所以这一条同时查运行中把 tick 改小
    sendCut(8'h1C, 4);
    delay(3000);
    send(8'h1C, False);
    waitFlag(3);
    rd(8'h@RXD@);
    action if (rdR[7:0] != 8'h1C || lastSt[1] != 0) begin $display("FAIL after a frame cut off at 4 bits and a 3 ms gap, the next frame reads %02h with rxerr %0d, want 1c and 0", rdR[7:0], lastSt[1]); bad <= True; end endaction
    wr(8'h@STATUS@, 32'h0F);
@TXPART@

    // 坏校验的帧：置 rxerr，不置 rxv
    send(8'h5A, True);
    waitFlag(3);
    rd(8'h@STATUS@);
    action
      Bool wrong = False;
      if (rdR[1] != 1) begin $display("FAIL a frame with even parity does not set rxerr"); wrong = True; end
      if (rdR[0] != 0) begin $display("FAIL a frame with even parity sets rxv"); wrong = True; end
      if (wrong) bad <= True;
    endaction
  endseq;

  FSM fsm <- mkFSM(test);
  Reg#(Bool) started <- mkReg(False);

  rule go (!started);
    started <= True;
    fsm.start;
  endrule

  rule tick_;
    cyc <= cyc + 1;
    if (cyc > 300000) begin
      $display("TIMEOUT");
      $finish(1);
    end
  endrule

  rule fin (started && fsm.done);
    if (bad) $display("FAILED");
    else $display("PASS ps2: @VERDICT@");
    $finish(bad ? 1 : 0);
  endrule
endmodule

endpackage
'''

txt = (TEMPLATE.replace("@L@", label)
       .replace("@TXON@", str(tx_on))
       .replace("@TX@", "True" if tx_on else "False")
       .replace("@TXPART@", tx_part)
       .replace("@VERDICT@", verdict)
       .replace("@CTRL@", f"{CTRL:02X}").replace("@TICK@", f"{TICK:02X}")
       .replace("@RXD@", f"{RXD:02X}").replace("@STATUS@", f"{STATUS:02X}"))

(out / f"Ps2{label}Tb.bsv").write_text(txt, encoding="utf-8")
print(f"  ps2 行为测试台就位：tx={tx_on}")
