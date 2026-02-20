# ntrip_client
Simple modules which can be used to enable NTRIP for RTK corrections without the need for bloated libraries and modules. 

I got tired of needing to use complicated 3rd party libraries which slowed down compilation and could have underlying issues which could not be fixed by me. As such, this module is intended to be extremely simple and should rely only on things which are part of the standard C++ installation in Linux. 

Currently the project is only intended for use on Linux, though support for Windows may be developed in the future if needed. It is confirmed to be building and running with a WSL installation using Ubuntu 20.04. Your mileage with other distros and installations may vary.

## Usage

To build the `ntrip_client`, you can run the following command:

```sh
./build.sh
```

To run the `ntrip_client`, after building you can run the following command:

```sh
./ntrip_client /path/to/grm.json
```

## FAQ

### What is NTRIP?

NTRIP stands for Networked Transport of RTCM via Internet Protocol. It is a protocol used to stream GNSS correction data over the internet.

### What are RTK corrections?

RTK (Real-Time Kinematic) corrections are used to improve the accuracy of GNSS positioning by providing real-time correction data.

### Do I need an internet connection to use this?

Yes, an internet connection is required to connect to the NTRIP caster and receive RTK corrections.

### Can I use this with any GNSS receiver?

This client is designed to work with GNSS receivers that support NTRIP for receiving RTK corrections.

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.

## Contact

For any questions or support, please open an issue on the GitHub repository.
