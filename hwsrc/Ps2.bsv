package Ps2;

// PS/2 主机。两根线先过两级同步，再按 Chapweske《The PS/2 Mouse/Keyboard Protocol》走：
//   收：设备出时钟，主机在下降沿采样，攒满 11 位（起始 0、8 位低位先出、奇校验、停止 1）
//   发：主机拉低时钟至少 100 微秒抑制通信，拉低数据、放开时钟作请求发送；设备出时钟，主机在
//       每个下降沿换一位，第 11 个时钟看设备拉没拉低数据作应答。设备 15 毫秒内不出时钟、
//       或者整包 2 毫秒内发不完，都算发送失败
// 节拍、收、发写的是同一批寄存器，所以合成一条规则；各分支先攒进局部变量，最后只写一次。

import RegIf::*;
import Ps2Regs::*;

typedef struct {
  Bool tx;
} Ps2Cfg;

interface Ps2Pins;
  (* always_ready, result = "clk_pull" *)  method Bit#(1) clk_pull;
  (* always_ready, result = "data_pull" *) method Bit#(1) data_pull;
  (* always_ready, always_enabled, prefix = "" *)
  method Action clk_in((* port = "clk_i" *) Bit#(1) v);
  (* always_ready, always_enabled, prefix = "" *)
  method Action data_in((* port = "data_i" *) Bit#(1) v);
endinterface

interface Ps2Ifc#(numeric type aw, numeric type dw);
  interface RegIf#(aw, dw) regs;
  interface Ps2Pins         pins;
  (* always_ready *) method Bool irq;
endinterface

typedef enum { Idle, Inhibit, Rts, Bits, Ack, Release } TxSt deriving (Bits, Eq);

module mkPs2#(Ps2Cfg cfg)(Ps2Ifc#(aw, dw))
    provisos (Mul#(TDiv#(dw, 8), 8, dw), Add#(_a, 8, aw), Add#(_b, 1, dw),
              Add#(_c, 8, dw), Add#(_d, 16, dw));

  Ps2RegsIfc#(aw, dw) r <- mkPs2Regs(Ps2RegsCfg { tx: cfg.tx });

  // 恒值的只读寄存器：特性关掉时占位，综合器整片消掉
  function Reg#(t) roReg(t v) = interface Reg;
    method t _read = v;
    method Action _write(t x) = noAction;
  endinterface;

  Wire#(Bit#(1)) clkIn  <- mkBypassWire;
  Wire#(Bit#(1)) dataIn <- mkBypassWire;
  // 写脉冲在总线方法之后才有，而引擎要在它之前读寄存器：隔一拍，由 CReg 递过去
  Reg#(Maybe#(Bit#(8))) pend[2] <- mkCReg(2, tagged Invalid);

  Reg#(Bit#(2))   cs    <- mkReg(2'b11);
  Reg#(Bit#(2))   dsy   <- mkReg(2'b11);
  Reg#(Bit#(1))   cPrev <- mkReg(1);
  Reg#(Bit#(16))  sub   <- mkReg(0);

  Reg#(UInt#(4))  rk    <- mkReg(0);
  Reg#(Bit#(10))  rsh   <- mkReg(0);
  Reg#(UInt#(12)) gap   <- mkReg(0);
  Reg#(Bit#(8))   rx    <- mkReg(0);

  Reg#(TxSt)      st    = roReg(Idle);
  Reg#(UInt#(16)) tc    = roReg(0);
  Reg#(UInt#(4))  tk    = roReg(0);
  Reg#(Bit#(10))  tsh   = roReg(0);
  Reg#(Bit#(1))   cPull = roReg(0);
  Reg#(Bit#(1))   dPull = roReg(0);
  if (cfg.tx) begin
    st    <- mkReg(Idle);
    tc    <- mkReg(0);
    tk    <- mkReg(0);
    tsh   <- mkReg(0);
    cPull <- mkReg(0);
    dPull <- mkReg(0);
  end

  rule mark;
    if (r.txd_wr) pend[1] <= tagged Valid r.txd_wr_val;
  endrule

  rule engine;
    Bit#(1) c  = cs[1];
    Bit#(1) dd = dsy[1];
    cs    <= {cs[0], clkIn};
    dsy   <= {dsy[0], dataIn};
    cPrev <= c;
    Bool fall = cPrev == 1 && c == 0;
    // 判「不小于」不判「等于」：tick 改小时计数可能已经越过新值，等于要等 16 位回绕
    Bool us   = sub >= r.tick;
    Bit#(16) nsub = us ? 0 : sub + 1;
    pend[0] <= tagged Invalid;

    UInt#(4)  nrk  = rk;
    Bit#(10)  nrsh = rsh;
    UInt#(12) ngap = gap;
    TxSt      nst  = st;
    UInt#(16) ntc  = tc;
    UInt#(4)  ntk  = tk;
    Bit#(1)   ncp  = cPull;
    Bit#(1)   ndp  = dPull;
    Bool rxOk = False;
    Bool rxBad = False;
    Bool txOk = False;
    Bool txBad = False;

    if (r.ctrl_en == 1) begin
      case (st)
        Idle: begin
          ncp = 0; ndp = 0;
          if (fall) begin
            ngap = 0;
            if (rk == 10) begin
              Bit#(11) f = {dd, rsh};
              if (f[0] == 0 && f[10] == 1 && (^f[9:1]) == 1) begin rx <= f[8:1]; rxOk = True; end
              else rxBad = True;
              nrk = 0;
            end else begin
              nrsh = {dd, rsh[9:1]};
              nrk = rk + 1;
            end
          end else if (us && rk != 0) begin
            // 两毫秒等不到下一个时钟，丢掉这半截帧，免得之后每一帧都错位
            if (gap == 2000) nrk = 0; else ngap = gap + 1;
          end
          // 时钟从这一拍就拉低、节拍从头数：引脚比状态晚一拍，节拍又可能数到一半，
          // 不这样抑制只有 99 微秒多一点，规范要求至少 100
          if (pend[0] matches tagged Valid .b &&& cfg.tx) begin
            tsh <= {1'b1, ~(^b), b};
            nst = Inhibit; ntc = 0; nrk = 0; ncp = 1; nsub = 0;
          end
        end
        // 抑制：拉低时钟 100 微秒
        Inhibit: begin
          ncp = 1;
          if (us) begin
            if (tc == 99) begin nst = Rts; ntc = 0; ncp = 0; ndp = 1; end
            else ntc = tc + 1;
          end
        end
        // 请求发送：数据拉着、时钟放开，等设备的第一个下降沿，换上第 0 位
        Rts: begin
          if (fall) begin ndp = ~tsh[0]; ntk = 1; nst = Bits; ntc = 0; end
          else if (us) begin
            if (tc == 14999) txBad = True; else ntc = tc + 1;
          end
        end
        // 每个下降沿换一位：第 1 到 7 位数据、校验、停止（停止位是 1，就是放开数据）
        Bits: begin
          if (fall) begin
            ndp = ~tsh[tk];
            if (tk == 9) nst = Ack;
            ntk = tk + 1;
          end
          if (us) begin
            if (tc == 1999) txBad = True; else ntc = tc + 1;
          end
        end
        // 第 11 个时钟：设备拉低数据就是应答
        Ack: begin
          if (fall) begin
            if (dd == 0) nst = Release; else txBad = True;
          end else if (us) begin
            if (tc == 1999) txBad = True; else ntc = tc + 1;
          end
        end
        Release: begin
          if (c == 1 && dd == 1) begin nst = Idle; txOk = True; end
        end
      endcase
      if (txBad) begin nst = Idle; ncp = 0; ndp = 0; end
    end else begin
      nst = Idle; ncp = 0; ndp = 0; nrk = 0;
    end

    sub   <= nsub;
    rk    <= nrk;
    rsh   <= nrsh;
    gap   <= ngap;
    st    <= nst;
    tc    <= ntc;
    tk    <= ntk;
    cPull <= ncp;
    dPull <= ndp;
    if (rxOk)  r.status_rxv_set(1);
    if (rxBad) r.status_rxerr_set(1);
    if (txOk)  r.status_txdone_set(1);
    if (txBad) r.status_txerr_set(1);
  endrule

  rule show;
    r.rxd_in(rx);
    r.status_busy_in(st != Idle ? 1 : 0);
  endrule

  interface regs = r.regs;
  interface Ps2Pins pins;
    method Bit#(1) clk_pull = cPull;
    method Bit#(1) data_pull = dPull;
    method Action clk_in(Bit#(1) v); clkIn._write(v); endmethod
    method Action data_in(Bit#(1) v); dataIn._write(v); endmethod
  endinterface
  method Bool irq = (r.status_rxv == 1 || r.status_rxerr == 1 ||
                     r.status_txdone == 1 || r.status_txerr == 1) && r.ctrl_ien == 1;
endmodule

endpackage
