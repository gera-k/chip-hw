#include "chip_hw.h"

#include <iostream>

#include "cxxopts.hpp"

using namespace std;

static int verbose = 0;

int main(int argc, char* argv[])
{

	try
	{
		cxxopts::Options options(argv[0], "C.H.I.P. hardware explorer ver 0.1 built " __DATE__ " " __TIME__);

		options.add_options()
			("h,help", "Print help")
			("V,verbose", "Print extra info")
			("P,pio", "Show Port Controller", cxxopts::value<string>()->implicit_value(""), "port[.pin]")
			;

		bool help = argc <= 1;

//		printf("argc=%d  help=%d\n", argc, help);

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
