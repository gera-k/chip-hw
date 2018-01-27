#ifndef _CHIP_HW_H_
#define _CHIP_HW_H_

#include <cstdint>
#include <stdexcept>

#include <fcntl.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/mman.h>

namespace R8
{
	static constexpr bool debug = !true;

	class RegBlock
	{
	private:
		uint32_t _addr;
		uint32_t _size;
		uint32_t _off;
		int _mem = -1;
		void* _map = MAP_FAILED;

		static constexpr uint32_t Page = 4096;
		static constexpr uint32_t Mask = Page - 1;

	public:
		RegBlock(uint32_t addr, uint32_t size)
		{
			_addr = addr & ~Mask;
			_off = addr & Mask;
			_size = size;

			if (debug) printf("addr %08x:%08X  off %08X  size %08X\n", addr, _addr, _off, _size);

			if (_off + _size > Page)
				throw std::runtime_error("invalid register block size");

			_mem = open("/dev/mem", O_RDWR | O_SYNC);
			if (_mem < 0)
				throw std::runtime_error("/dev/mem open error");

			_map = mmap(0, Page, PROT_READ | PROT_WRITE, MAP_SHARED, _mem, _addr);
			if (_map == MAP_FAILED)
				throw std::runtime_error("/dev/mem mmap error");

		}

		~RegBlock()
		{
			if (_map != MAP_FAILED)
				munmap(_map, _size);

			if (_mem > 0)
				close(_mem);
		}

		using Reg = uint32_t;

		uint32_t reg(Reg reg)
		{
			if (reg >= _size)
				throw std::runtime_error("invalid register offset");

			uint32_t d = *(uint32_t*)((uint8_t*)_map + _off + reg);
			if (debug) printf("reg[%08X]:%08X\n", _addr + _off + reg, d);
			return d;
		}

		void reg(Reg reg, uint32_t val)
		{
			if (reg >= _size)
				throw std::runtime_error("invalid register offset");

			if (debug) printf("reg[%08X]=%08X\n", _addr + _off + reg, val);
			*(uint32_t*)((uint8_t*)_map + _off + reg) = val;
		}

	};

	// Port controller
	//	0x01C20800---0x01C20BFF 1K
	class PIO : public RegBlock
	{
	public:

		PIO() : RegBlock(0x01C20800, 1024)
		{}

		using Pin = int8_t;

		// ports
		enum Port
		{
			PA,
			PB,
			PC,
			PD,
			PE,
			PF,
			PG,

			PortMin = PA,
			PortMax = PG,

			PortNone = -1
		};

		class PP
		{
		private:
			int8_t _port;
			int8_t _pin;
		public:
			PP(Port port, Pin pin) : _port(port), _pin(pin) {}
			Port port() { return Port(_port); }
			Pin pin() { return Pin(_pin); }
		};

		static std::string toStr(Port port)
		{
			switch(port)
			{
			case PA: return "PA"; break;
			case PB: return "PB"; break;
			case PC: return "PC"; break;
			case PD: return "PD"; break;
			case PE: return "PE"; break;
			case PF: return "PF"; break;
			case PG: return "PG"; break;
			default: break;
			}

			return "??";
		}
		static Port toPort(const std::string& s)
		{
			if (s.compare("PA") == 0) return PA;
			if (s.compare("PB") == 0) return PB;
			if (s.compare("PC") == 0) return PC;
			if (s.compare("PD") == 0) return PD;
			if (s.compare("PE") == 0) return PE;
			if (s.compare("PF") == 0) return PF;
			if (s.compare("PG") == 0) return PG;
			throw std::runtime_error("Invalid port");
		}


		static constexpr Pin PinMin = 0;
		static constexpr Pin PinMax = 31;
		static constexpr Pin PinNone = -1;

		static std::string toStr(Pin pin)
		{
			return std::to_string(uint8_t(pin));
		}
		static Pin toPin(const std::string& s)
		{
			int i = std::stoi(s);
			if (i >= PinMin && i <= PinMax)
				return Pin(i);
			throw std::runtime_error("Invalid pin");
		}

		// register within a port
		enum Reg
		{
			CFG0,
			CFG1,
			CFG2,
			CFG3,
			DATA,
			DRV0,
			DRV1,
			PUL0,
			PUL1
		};

		static std::string toStr(Reg reg)
		{
			switch(reg)
			{
			case CFG0: return "CFG0"; break;
			case CFG1: return "CFG1"; break;
			case CFG2: return "CFG2"; break;
			case CFG3: return "CFG3"; break;
			case DATA:  return "DATA"; break;
			case DRV0: return "DRV0"; break;
			case DRV1: return "DRV1"; break;
			case PUL0: return "PUL0"; break;
			case PUL1: return "PUL1"; break;
			}

			return "????";
		}

		// pin function
		enum Func
		{
			M0,
			M1,
			M2,
			M3,
			M4,
			M5,
			M6,
			Input = M0,
			Output = M1,
		};

		static std::string toStr(Func func)
		{
			switch(func)
			{
			case M0: return "Input"; break;
			case M1: return "Output"; break;
			case M2: return "M2"; break;
			case M3: return "M3"; break;
			case M4: return "M4"; break;
			case M5: return "M5"; break;
			case M6: return "M6"; break;
			}

			return "??";
		}

		// pin driver level
		enum Drv
		{
			L10,
			L20,
			L30,
			L40
		};

		static std::string toStr(Drv drv)
		{
			switch(drv)
			{
			case L10: return "10mA"; break;
			case L20: return "20mA"; break;
			case L30: return "30mA"; break;
			case L40: return "40mA"; break;
			}

			return "??";
		}

		enum Pull
		{
			Pull_disable,
			Pull_up,
			Pull_down
		};

		static std::string toStr(Pull pull)
		{
			switch(pull)
			{
			case Pull_disable: return "Disabled"; break;
			case Pull_up: return "Up"; break;
			case Pull_down: return "Down"; break;
			}

			return "??";
		}

		static constexpr uint32_t Off(Port p, Reg r)
		{
			return p * 0x24 + r * 4;
		}

		// true if pin exists in port
		static bool Bit(Port p, Pin pin)
		{
			static const uint32_t Bits[] =
			{
					0x00000000,		// A
					0x0007841F,		// B
					0x0008FFFF,		// C
					0x0FFCFCFC,		// D
					0x00000FFF,		// E
					0x0000003F,		// F
					0x00001E1F,		// E
			};
			return (pin < 32) && (Bits[p] & (1<<pin));
		}

		// read/write pin value
		bool Value(Port port, Pin pin)
		{
			if (!Bit(port, pin))
				throw std::runtime_error("invalid pin number");

			uint32_t d = reg(Off(port, DATA));	// read DATA register

			return d & (1 << pin);
		}
		void Value(Port port, Pin pin, bool val)
		{
			if (!Bit(port, pin))
				throw std::runtime_error("invalid pin number");

			uint32_t d = reg(Off(port, DATA));	// read DATA register

			if (val)
				d |= (1 << pin);
			else
				d &= ~(1 << pin);

			reg(Off(port, DATA), d);
		}

		// read/write pin function
		Func Function(Port port, Pin pin)
		{
			if (!Bit(port, pin))
				throw std::runtime_error("invalid pin number");

			Reg r = Reg(CFG0 + pin / 8);	// 8 ping per register
			uint8_t s = (pin % 8) * 4;		// shift to corresponding bits

			uint32_t d = reg(Off(port, r));	// read register
			return Func((d >> s) & 0x7);	// return function
		}
		Func Function(PP pp)
		{
			return Function(pp.port(), pp.pin());
		}
		void Function(Port port, Pin pin, Func func)
		{
			if (!Bit(port, pin))
				throw std::runtime_error("invalid pin number");

			if (func > 6)
				throw std::runtime_error("invalid pin function");

			Reg r = Reg(CFG0 + pin / 8);	// 8 ping per register
			uint8_t s = (pin % 8) * 4;		// shift to corresponding bits

			uint32_t d = reg(Off(port, r));	// read register

			d &= ~(0xF << s);	// clear function bits
			d |= (func << s);	// set new function

			reg(Off(port,r), d);
		}
		void Function(PP pp, Func func)
		{
			Function(pp.port(), pp.pin(), func);
		}

		// read/write Pin driver level
		Drv Driver(Port port, Pin pin)
		{
			if (!Bit(port, pin))
				throw std::runtime_error("invalid pin number");

			Reg r = Reg(DRV0 + pin / 16);	// 16 ping per register
			uint8_t s = (pin % 16) * 2;		// shift to corresponding bits

			uint32_t d = reg(Off(port, r));	// read register
			return Drv((d >> s) & 0x3);		// return level
		}
		void Driver(Port port, Pin pin, Drv drv)
		{
			if (!Bit(port, pin))
				throw std::runtime_error("invalid pin number");

			if (drv > 3)
				throw std::runtime_error("invalid driver level");

			Reg r = Reg(DRV0 + pin / 16);	// 16 ping per register
			uint8_t s = (pin % 16) * 2;		// shift to corresponding bits

			uint32_t d = reg(Off(port, r));	// read register

			d &= ~(0x3 << s);	// clear function bits
			d |= (drv << s);	// set new level

			reg(Off(port,r), d);
		}

		// read/write Pull Up/Down
		Pull PullUpDown(Port port, Pin pin)
		{
			if (!Bit(port, pin))
				throw std::runtime_error("invalid pin number");

			Reg r = Reg(PUL0 + pin / 16);	// 16 ping per register
			uint8_t s = (pin % 16) * 2;		// shift to corresponding bits

			uint32_t d = reg(Off(port, r));	// read register
			return Pull((d >> s) & 0x3);		// return level
		}
		void PullUpDown(Port port, Pin pin, Pull drv)
		{
			if (!Bit(port, pin))
				throw std::runtime_error("invalid pin number");

			if (drv > 2)
				throw std::runtime_error("invalid Pull Up/Down value");

			Reg r = Reg(PUL0 + pin / 16);	// 16 ping per register
			uint8_t s = (pin % 16) * 2;		// shift to corresponding bits

			uint32_t d = reg(Off(port, r));	// read register

			d &= ~(0x3 << s);	// clear function bits
			d |= (drv << s);	// set new level

			reg(Off(port,r), d);
		}

	};

	// CCM
	//	0x01C20000 1024
	class CCM : public RegBlock
	{
	private:
		enum Reg
		{
			PLL1_CFG = 0x0000, // PLL1 Control
			PLL1_TUN = 0x0004, // PLL1 Tuning
			PLL2_CFG = 0x0008, // PLL2 Control
			PLL2_TUN = 0x000C, // PLL2 Tuning
			PLL3_CFG = 0x0010, // PLL3 Control
			PLL4_CFG = 0x0018, // PLL4 Control
			PLL5_CFG = 0x0020, // PLL5 Control
			PLL5_TUN = 0x0024, // PLL5 Tuning
			PLL6_CFG = 0x0028, // PLL6 Control
			PLL6_TUN = 0x002C, // PLL6 Tuning
			PLL7_CFG = 0x0030, // /
			PLL1_TUN2 = 0x0038, // PLL1 Tuning2
			PLL5_TUN2 = 0x003C, // PLL5 Tuning2
			OSC24M_CFG = 0x0050, // OSC24M control
			CPU_AHB_APB0 = 0x0054, // CPU, AHB And APB0 Divide Ratio
			APB1_CLK_DIV = 0x0058, // APB1 Clock Divider
			AXI_GATING = 0x005C, // AXI Module Clock Gating
			AHB_GATING0 = 0x0060, // AHB Module Clock Gating 0
			AHB_GATING1 = 0x0064, // AHB Module Clock Gating 1
			APB0_GATING = 0x0068, // APB0 Module Clock Gating
			APB1_GATING = 0x006C, // APB1 Module Clock Gating
			NAND_SCLK_CFG = 0x0080, // Nand Flash Clock
			SD0_SCLK_CFG = 0x0088, // SD0 Clock
			SD1_SCLK_CFG = 0x008C, // SD1 Clock
			SD2_SCLK_CFG = 0x0090, // SD2 Clock
			CE_SCLK_CFG = 0x009C, // Crypto Engine Clock
			SPI0_SCLK_CFG = 0x00A0, // SPI0 Clock
			SPI1_SCLK_CFG = 0x00A4, // SPI1 Clock
			SPI2_SCLK_CFG = 0x00A8, // SPI2 Clock
			IR_SCLK_CFG = 0x00B0, // IR Clock
			DRAM_SCLK_CFG = 0x0100, // DRAM Clock
			BE_CFG = 0x0104, // Display Engine Backend Clock
			FE_CFG = 0x010C, // Display Engine Front End Clock
			LCD_CH1_CFG = 0x012C, // LCD Channel1 Clock
			CSI_CFG = 0x0134, // CSI Clock
			VE_CFG = 0x013C, // Video Engine Clock
			AUDIO_CODEC_SCLK_CFG = 0x0140, // Audio Codec Gating Special Clock
			AVS_SCLK_CFG = 0x0144, // AVS Gating Special Clock
			MALI_CLOCK_CFG = 0x0154, // Mali400 Gating Special Clock
			MBUS_SCLK_CFG = 0x015C, // MBUS Gating Clock
			IEP_SCLK_CFG = 0x0160, // IEP Gating Clock
		};

		enum RegAHB_GATING0
		{
			STIMER_AHB_GATING = (1<<28),
			SPI2_AHB_GATING = (1<<22),
			SPI1_AHB_GATING = (1<<21),
			SPI0_AHB_GATING = (1<<20),
			SDRAM_AHB_GATING = (1<<14),
			NAND_AHB_GATING = (1<<13),
			SD2_AHB_GATING = (1<<10),
			SD1_AHB_GATING = (1<<9),
			SD0_AHB_GATING = (1<<8),
			BIST_AHB_GATING = (1<<7),
			DMA_AHB_GATING = (1<<6),
			CE_AHB_GATING = (1<<5),
			OHCI_AHB_GATING = (1<<2),
			EHCI_AHB_GATING = (1<<1),
			USBOTG_AHB_GATING = (1<<0),
		};

	public:
		CCM() : RegBlock(0x01C20000, 1024) {}

		// enable/disable spi2 clock
		//	source: 0 - OSC24, 1 - PLL6, 2 - PLL5
		// SPI clock = source/2^n/m
		void spi2(uint32_t n, uint32_t m, uint32_t source = 0)
		{
			uint32_t d = (1<<31);	// enable
			d |= (source & 3) << 24;
			d |= (n & 3) << 16;
			d |= ((m-1) & 0xF) << 0;
			reg(SPI2_SCLK_CFG, d);

			reg(AHB_GATING0, reg(AHB_GATING0) | SPI2_AHB_GATING);
		}
		void spi2()
		{
			reg(AHB_GATING0, reg(AHB_GATING0) & ~SPI2_AHB_GATING);
			reg(SPI2_SCLK_CFG, 0);
		}
	};

	// PWM
	//	0x01C20E00---0x01C20E08 8 bytes
	class PWM : public RegBlock
	{
	private:
		static constexpr PIO::Port _port = PIO::PB;
		static constexpr PIO::Pin _pin = 2;
		static constexpr PIO::Func Pwm = PIO::M2;

		// registers
		enum Reg
		{
			CTRL = 0,
			PERIOD = 4
		};

		// Control register
		enum RegCtrl
		{
			RDY = (1<<28),		// 0 - Period reg is ready to write
								// 1 = Period reg is busy
			BYPASS = (1<<9),	// 1 - enable 24MHz clock
			PUL_START = (1<<8),	// 1 - output one pulse
			MODE = (1<<7),		// 0 - cycle mode
								// 1 - pulse mode
			GATING = (1<<6),	// 0 - mask special clock
								// 1 - pass
			ACT_STA = (1<<5),	// 0 - Active low
								// 1 - high
			EN = (1<<4),		// 1 - channel enable
		};

	public:

		using Length = uint16_t;	// length of period/duty cycle in clocks

		enum Scale
		{
			SCALE_120 = 0,		// scale = 1/120
			SCALE_180 = 1,
			SCALE_240 = 2,
			SCALE_360 = 3,
			SCALE_480 = 4,
			SCALE_12K = 8,
			SCALE_24K = 9,
			SCALE_36K = 10,
			SCALE_48K = 11,
			SCALE_72K = 12,
			SCALE_1 = 15,
		};

		PWM() : RegBlock(0x01C20E00, 8)
		{
			// set pin function to PWM
			PIO pio;
			pio.Function(_port, _pin, Pwm);

			// TODO: driver level, pull

			stop();
		}

		~PWM()
		{
			stop();

			PIO pio;
			pio.Function(_port, _pin, PIO::Input);
		}

		void start(Scale scale, Length period, Length duty_cycle)
		{
			uint32_t d;

			d = period - 1;
			d <<= 16;
			d |= duty_cycle;
			reg(PERIOD, d);

			d = scale;
			d |= GATING | ACT_STA | EN;
			reg(CTRL, d);
		}

		void stop()
		{
			reg(CTRL, 0);
		}

	};

	// SPI
	//	0: 0x01C05000 0x2C bytes
	//	1: 0x01C06000
	//	2: 0x01C17000
	class SPI : public RegBlock
	{
	private:
		// registers
		enum Reg : RegBlock::Reg
		{
			RXDATA = 0x00,		// RX Data
			TXDATA = 0x04,		// TX Data
			CTL = 0x08,			// Control
			INTCTL = 0x0C,		// Interrupt Control
			ST = 0x10,			// Status
			DMACTL = 0x14,		// DMA Control
			WAIT = 0x18,		// Wait Clock Counter
			CCTL =  0x1C,		// Clock rate control
			BCNT = 0x20,		// Burst Counter
			TCNT = 0x24,		// Transmit Counter
			FIFO_STA = 0x28		// FIFO Status
		};

		// CTL - Control register
		enum RegCTL
		{
			MSDL = (1<<19),		// Master Sample Data Control
			TPE = (1<<18),		// Transmit Pause Enable
			SS_LEVEL = (1<<17),	// SS level manual control
			SS_CTRL = (1<<16),	// SS Mode select (0 - auto, 1 - manual)
			DHB = (1<<15),		// Discard Hash Burst
			DDB = (1<<14),		// Dummy Burst type
			SS0 = (0<<12),		// use SPI_SS0
			SS1 = (1<<12),		// use SPI_SS1
			SS2 = (2<<12),		// use SPI_SS2
			SS3 = (3<<12),		// use SPI_SS3
			RPSM = (1<<11),		// Rapids mode select
			XCH = (1<<10),		// Exchange burst
			RX_RESET = (1<<9),	// RX FIFO reset
			TX_RESET = (1<<8),	// TX FIFO reset
			SSCTL = (1<<7),		// Negate SPI_SSx between bursts
			LMTF = (1<<6),		// LSB/MSB transfer First select (0 - MSB, 1 - LSB)
			DMAM = (1<<5),		// DMA mode
			SSPOL = (1<<4),		// SS polarity (0 - active high, 1 - active low)
			POL = (1<<3),		// CLK polarity (0 - active high, 1 - active low)
			PHA = (1<<2),		// Clock/data phase control
			MODE = (1<<1),		// Mode (0 - slave, 1 - master)
			EN = (1<<0),		// Enable
		};

		// STA Status register
		enum RegSTA
		{
			SSI = (1<<17),		// SS invalid
			TC = (1<<16),		// Transfer complete
			TU = (1<<14),		// TX FIFO underrun
			TO = (1<<13),		// TX FIFO overflow
			T34 = (1<<12),		// TX FIFO 3/4 empty
			T14 = (1<<11),		// TX FIFO 1/4 empty
			TF = (1<<10),		// TX FIFO full
			THE = (1<<9),		// TX FIFO half empty
			TE = (1<<8),		// TX FIFO Empty
			RU = (1<<6),		// RX FIFO Underrun
			RO = (1<<5),		// RX FIFO Overflow
			R34 = (1<<4),		// RX FIFO 3/4 full
			R14 = (1<<3),		// RX FIFO 1/4 full
			RF = (1<<2),		// RX FIFO full
			RHF = (1<<1),		// RX FIFO Half full
			RR = (1<<0),		// RX FIFO ready
		};

		// CCTL Clock Control
		enum RegCCTL
		{
			DRS = (1<<12),		// Divide rate select (0 - rate 1, 1 - rate 2)
			CDR1_MASK = (0xF << 8),	// CDR1: clock = AHB_CLK/2^(n+1)
			CDR1_SHIFT = 8,
			CDR2_MASK = 0xFF,		// CDR2: clock = AHB_CLK/(2*(n+1))
			CDR2_SHIFT = 0,
		};

		// FIFO_STA  FIFO Status
		enum RegFIFO
		{
			TX_MASK = (0x3F << 16),
			TX_SHIFT = 16,
			RX_MASK = (0x3F << 0),
			RX_SHIFT = 0,
		};

		int _module;
		uint32_t _mode = SSPOL | MODE | EN;		// default mode bits of the CTL reg

		PIO::PP CLK[3] =  { {PIO::PC, 2}, {PIO::PG, 10}, {PIO::PE, 1} };
		PIO::PP MOSI[3] = { {PIO::PC, 0}, {PIO::PG, 11}, {PIO::PE, 2} };
		PIO::PP MISO[3] = { {PIO::PC, 1}, {PIO::PG, 12}, {PIO::PE, 3} };
		PIO::PP CS0[3] =  { {PIO::PC, 3}, {PIO::PG,  9}, {PIO::PE, 0} };
		PIO::Func FN[3] = { PIO::M3,      PIO::M2,       PIO::M4      };

	public:

		SPI(int module) :
			RegBlock(
					module == 0 ? 0x01C05000 :
					module == 1 ? 0x01C06000 :
					module == 2 ? 0x01C17000 : 0,
					0x2C),
			_module(module)
		{
			// enable clock
			if (module == 2)
				CCM().spi2(3,3);	// 24/8/3 = 1MHz

			// set pin functions to SPI
			PIO pio;
			pio.Function(CLK[_module], FN[_module]);
			pio.Function(MOSI[_module], FN[_module]);
			pio.Function(MISO[_module], FN[_module]);
			pio.Function(CS0[_module], FN[_module]);

			// TODO: driver level, pull

			reg(CTL, _mode);
		}

		~SPI()
		{
			reg(CTL, 0);

			PIO pio;
			pio.Function(CLK[_module], PIO::Input);
			pio.Function(MOSI[_module], PIO::Input);
			pio.Function(MISO[_module], PIO::Input);
			pio.Function(CS0[_module], PIO::Input);

			CCM().spi2();
		}

		void mode()
		{
			// TODO: set controller mode(s)
		}

		void clock(uint32_t div, bool cdr2 = false)
		{
			uint32_t d = cdr2 ? DRS : 0;
			if (cdr2)
				d |= ((div << CDR2_SHIFT) & CDR2_MASK);
			else
				d |= ((div << CDR1_SHIFT) & CDR1_MASK);
			reg(CCTL, d);
		}

		void tx(uint32_t data)
		{
			// clear status
			reg(ST, ~0);

			// reg(CTL, reg(CTL) | DHB );

			// fill FIFO
			reg(TXDATA, data);

			// set length
			reg(BCNT, 4);
			reg(TCNT, 4);

			// start transfer
			reg(CTL, reg(CTL) | XCH);

			// wait for completion
			uint32_t t = 1000000;
			while (!(reg(ST) & TC) && (t--));

			reg(CTL, _mode);
		}
	};

	// LRADC - low resolution ADC
	class LRADC : public RegBlock
	{
	private:
		enum Reg
		{
			CTRL = 0x00,
			INTC = 0x04,
			INTS = 0x08,
			DATA0 = 0x0C,
			DATA1 = 0x10
		};

		enum RegCTRL
		{
			CHAN_0 = (0 << 22),				// select channel 0
			CHAN_1 = (1 << 22),				// select channel 1
			CTS_SHIFT = 16,					// continuous time select
			CTS_MASK = (0xF << CTS_SHIFT),	//	update on every 8*(CTS+1) sample
			KEY_MODE_NORNAL = (0 << 12),
			KEY_MODE_SINGLE = (1 << 12),
			KEY_MODE_CONT = (2 << 12),
			RATE_250 = (0 << 2),
			RATE_125 = (1 << 2),
			RATE_62p5 = (2 << 2),
			RATE_32p25 = (3 << 2),
			EN = 1
		};

		enum RegINTS
		{
			ADC0_DATA_PENDING = (1 << 0)
		};

	public:
		LRADC() : RegBlock(0x01C22800, 0x14)
		{
			reg(CTRL, CHAN_0 | KEY_MODE_CONT | RATE_250 | EN);
		}

		~LRADC()
		{
			reg(CTRL, 0);
		}

		uint8_t data()
		{
			int t = 1000000;
			while ( !(reg(INTS) & ADC0_DATA_PENDING) && (t--));
			reg(INTS, ~0);
			return reg(DATA0) & 0x3F;
		}
	};
}



#endif
