#include "chip_hw.h"

#include <iostream>

#include "cxxopts.hpp"

using namespace std;

static int verbose = 0;

int main(int argc, char* argv[])
{

	try
	{
		vector<uint32_t> args;

		cxxopts::Options options(argv[0], "C.H.I.P. hardware explorer ver 0.1 built " __DATE__ " " __TIME__);

		options.add_options()
			("h,help", "Print help")
			("V,verbose", "Print extra info")
			("P,pio", "Show Port Controller pin(s)", cxxopts::value<string>()->implicit_value(""), "port[.pin]")
			("p,pin", "Control PIO pin ([{-r|-s|-c} [-d] [-r])", cxxopts::value<string>(), "port.pin")
			("i,input", "Set pin to input and read its value")
			("s,set", "Set pin to output and write 1 to it")
			("c,clear", "Set pin to output and write 0 to it")
			("f,function", "Set pin function (0..6)", cxxopts::value<int>(), "func")
			("d,driver", "Set pin driver level (0..3)", cxxopts::value<int>(), "level")
			("r,resistor", "Set pin pull up/down resistor (0-no, 1-up, 2-down)", cxxopts::value<int>(), "mode")
			("pwm", "Start PWM", cxxopts::value<int>(), "period [duty]")
			("spi", "Send data to SPI2", cxxopts::value<int>(), "div data [data...]")
			("positional", "", cxxopts::value<vector<uint32_t>>(args))
			;

		bool help = argc <= 1;

//		printf("argc=%d  help=%d\n", argc, help);

		options.parse_positional({"positional"});
		auto result = options.parse(argc, argv);

//		printf("help=%d\n", result.count("help"));
//		printf("pio=%d\n", result.count("pio"));

		if (help || result.count("help"))
		{
			cout << options.help() << endl;
			return 0;
		}

		verbose = result.count("verbose");

		if (result.count("pio"))
		{
			R8::PIO pio;

			auto arg = result["pio"].as<string>();

			R8::PIO::Port arg_port = pio.PortNone;
			R8::PIO::Pin arg_pin = pio.PinNone;

			if (!arg.empty())
			{
				auto p = arg.find('.');
				if (p == string::npos)
				{
					arg_port = pio.toPort(arg);
				}
				else
				{
					arg_port = pio.toPort(arg.substr(0,p));
					arg_pin = pio.toPin(arg.substr(p+1));
				}
			}

			for (R8::PIO::Port port = pio.PortMin; port <= pio.PortMax; port = R8::PIO::Port(port + 1))
			{
				if (arg_port != pio.PortNone && arg_port != port)
					continue;

				for (uint8_t pin = pio.PinMin; pin <= pio.PinMax; pin++)
				{
					if (arg_pin != pio.PinNone && arg_pin != pin)
						continue;

					if (!pio.Bit(port, pin))
						continue;

					printf("%s.%02d: %s  Func: %s  Driver: %s  Pull: %s\n",
						pio.toStr(port).c_str(), pin,
						pio.Value(port,pin) ? "On" : "Off",
						pio.toStr(pio.Function(port,pin)).c_str(),
						pio.toStr(pio.Driver(port,pin)).c_str(),
						pio.toStr(pio.PullUpDown(port,pin)).c_str()
						);
				}
			}
		}
		else if (result.count("pin"))
		{
			R8::PIO pio;

			auto arg = result["pin"].as<string>();

			auto p = arg.find('.');
			if (p == string::npos)
				throw runtime_error("Invalid port/pin");

			R8::PIO::Port port = pio.toPort(arg.substr(0,p));
			R8::PIO::Pin pin = pio.toPin(arg.substr(p+1));

			if (!pio.Bit(port, pin))
				throw runtime_error("Invalid port/pin");

			if (result.count("driver"))
			{
				int d = result["driver"].as<int>();
				pio.Driver(port, pin, R8::PIO::Drv(d));
			}

			if (result.count("function"))
			{
				int d = result["function"].as<int>();
				pio.Function(port, pin, R8::PIO::Func(d));
			}

			if (result.count("resistor"))
			{
				int d = result["resistor"].as<int>();
				pio.PullUpDown(port, pin, R8::PIO::Pull(d));
			}

			if (result.count("set"))
			{
				pio.Function(port,pin, pio.Output);
				pio.Value(port, pin, true);
			}
			else if (result.count("clear"))
			{
				pio.Function(port,pin, pio.Output);
				pio.Value(port, pin, false);
			}
			else if (result.count("input"))
			{
				pio.Function(port,pin, pio.Input);
				pio.Value(port, pin, false);
			}

			printf("%s.%02d: %s  Func: %s  Driver: %s  Pull: %s\n",
				pio.toStr(port).c_str(), pin,
				pio.Value(port,pin) ? "On" : "Off",
				pio.toStr(pio.Function(port,pin)).c_str(),
				pio.toStr(pio.Driver(port,pin)).c_str(),
				pio.toStr(pio.PullUpDown(port,pin)).c_str()
				);
		}
		else if (result.count("pwm"))
		{
			auto arg0 = result["pwm"].as<int>();

			R8::PWM::Length period = arg0;
			R8::PWM::Length duty_cycle = period/2;

			if (args.size() > 0)
				duty_cycle = args[0];

			R8::PWM pwm;

			pwm.start(pwm.SCALE_1, period, duty_cycle);

			printf("Hit any key to stop...");
			getchar();

		}
		else if (result.count("spi"))
		{
			auto div = result["spi"].as<int>();
			uint32_t d = 0xAAAAAAAA;

			if (args.size() > 0)
				d = args[0];

			R8::SPI spi(2);
			spi.clock(div);

			while(true)
			{
				spi.tx(d);

				printf("Hit any key to repeat, q to quit...");
				if (getchar() == 'q')
					break;
			}
		}

	}
	catch (const cxxopts::OptionException& e)
	{
				cout << "error parsing options: " << e.what() << endl;
						return 1;
	}
	catch (const runtime_error& e)
	{
				cout << "error: " << e.what() << "  errno: " << errno << endl;
						return 2;
	}
	catch (const exception& e)
	{
				cout << "error: " << e.what() << endl;
						return 3;
	}

	return 0;
}
