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
			if (size > Page)
				throw std::runtime_error("invalid register block size");

			_mem = open("/dev/mem", O_RDWR | O_SYNC);
			if (_mem < 0)
				throw std::runtime_error("/dev/mem open error");

			_addr = addr & ~Mask;
			_off = addr & Mask;
			_size = size;

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

		uint32_t reg(uint32_t off)
		{
			if (off >= _size)
				throw std::runtime_error("invalid register offset");

			return *(uint32_t*)((uint8_t*)_map + _off + off);
		}

		void reg(uint32_t off, uint32_t val)
		{
			if (off >= _size)
				throw std::runtime_error("invalid register offset");

			*(uint32_t*)((uint8_t*)_map + _off + off) = val;
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
			L0,
			L1,
			L2,
			L3
		};

		static std::string toStr(Drv drv)
		{
			switch(drv)
			{
			case L0: return "L0"; break;
			case L1: return "L1"; break;
			case L2: return "L2"; break;
			case L3: return "L3"; break;
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
}



#endif
