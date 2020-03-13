/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include <px4_arch/spi_hw_description.h>
#include <drivers/drv_sensor.h>
#include <nuttx/spi/spi.h>

constexpr px4_spi_bus_t px4_spi_buses[SPI_BUS_MAX_BUS_ITEMS] = {
	initSPIBus(SPI::Bus::SPI1, {
		initSPIDevice(DRV_IMU_DEVTYPE_ICM20689, SPI::CS{GPIO::PortF, GPIO::Pin2}, SPI::DRDY{GPIO::PortB, GPIO::Pin4}),
		initSPIDevice(DRV_IMU_DEVTYPE_ICM20602, SPI::CS{GPIO::PortF, GPIO::Pin3}, SPI::DRDY{GPIO::PortC, GPIO::Pin5}),
		initSPIDevice(DRV_GYR_DEVTYPE_BMI055, SPI::CS{GPIO::PortF, GPIO::Pin4}, SPI::DRDY{GPIO::PortB, GPIO::Pin14}),
		initSPIDevice(DRV_ACC_DEVTYPE_BMI055, SPI::CS{GPIO::PortG, GPIO::Pin10}, SPI::DRDY{GPIO::PortB, GPIO::Pin15}),
	}, {GPIO::PortE, GPIO::Pin3}),
	initSPIBus(SPI::Bus::SPI2, {
		initSPIDevice(SPIDEV_FLASH(0), SPI::CS{GPIO::PortF, GPIO::Pin5})
	}),
	initSPIBus(SPI::Bus::SPI4, {
		initSPIDevice(DRV_BARO_DEVTYPE_MS5611, SPI::CS{GPIO::PortF, GPIO::Pin10}),
	}),
	initSPIBusExternal(SPI::Bus::SPI5, {
		initSPIConfigExternal(SPI::CS{GPIO::PortI, GPIO::Pin4}, SPI::DRDY{GPIO::PortD, GPIO::Pin15}),
		initSPIConfigExternal(SPI::CS{GPIO::PortI, GPIO::Pin10}),
		initSPIConfigExternal(SPI::CS{GPIO::PortI, GPIO::Pin11})
	}),
	initSPIBusExternal(SPI::Bus::SPI6, {
		initSPIConfigExternal(SPI::CS{GPIO::PortI, GPIO::Pin6}),
		initSPIConfigExternal(SPI::CS{GPIO::PortI, GPIO::Pin7}),
		initSPIConfigExternal(SPI::CS{GPIO::PortI, GPIO::Pin8})
	}),
};

static constexpr bool unused = validateSPIConfig(px4_spi_buses);

#include <lib/drivers/device/spi.h>
#include <lib/drivers/accelerometer/PX4Accelerometer.hpp>

extern uint8_t s[8 * 1024];
extern uint8_t d[8 * 1024];


class testSPI : public device::SPI
{
public:

	/**
	* Constructor
	*
	* @param name    Driver name
	* @param devname Device node name
	* @param bus   SPI bus on which the device lives
	* @param device  Device handle (used by SPI_SELECT)
	* @param mode    SPI clock/data mode
	* @param frequency SPI clock frequency
	*/
	testSPI(const char *name, const char *devname, int bus, uint32_t device, enum spi_mode_e mode, uint32_t frequency);

	virtual ~testSPI() {};

	int init() { return SPI::init(); }



	/**
	 * Perform a SPI transfer.
	 *
	 * If called from interrupt context, this interface does not lock
	 * the bus and may interfere with non-interrupt-context callers.
	 *
	 * Clients in a mixed interrupt/non-interrupt configuration must
	 * ensure appropriate interlocking.
	 *
	 * At least one of send or recv must be non-null.
	 *
	 * @param send    Bytes to send to the device, or nullptr if
	 *      no data is to be sent.
	 * @param recv    Buffer for receiving bytes from the device,
	 *      or nullptr if no bytes are to be received.
	 * @param len   Number of bytes to transfer.
	 * @return    OK if the exchange was successful, -errno
	 *      otherwise.
	 */
	int   transfer(uint8_t *send, uint8_t *recv, unsigned len)
	{
		return SPI::transfer(send, recv, len);
	}
	uint32_t  get_actual_frequency()
	{
		return SPI::get_actual_frequency();
	}

};

testSPI::testSPI(const char *name, const char *devname, int bus, uint32_t device, enum spi_mode_e mode,
		 uint32_t frequency) :
	SPI::SPI(name, devname, bus, device, mode, frequency)
{
}


extern "C" void do_test(void);

__EXPORT  void do_test(void)
{
//  SPI::SPI(const char *name, const char *devname, int bus, uint32_t device, enum spi_mode_e mode, uint32_t frequency) :

	testSPI *spi = new testSPI("test", "/dev/testspi", 5, 0, SPIDEV_MODE3, 20 * 1000 * 1000);
	spi->init();

//  int   transfer(uint8_t *send, uint8_t *recv, unsigned len);
	for (uint32_t i = 0; i < sizeof(s); i++) {
		s[i] = i;
		d[i] = 0;
	}

	PROBE(1, 1);
	PROBE(2, 1);
	PROBE(3, 1);
	PROBE(4, 1);
	PROBE(5, 1);
	PROBE(6, 1);
	PROBE(7, 1);
	PROBE(1, 0);
	spi->transfer(s, d, 1);
	spi->transfer(s, d, 2);
	spi->transfer(s, d, 3);
	spi->transfer(s, d, 4);
	spi->transfer(s, d, 5);
	spi->transfer(s, d, 6);
	spi->transfer(s, d, 7);
	spi->transfer(s, d, 8);
	spi->transfer(s, d, 9);
	spi->transfer(s, d, 10);
	spi->transfer(s, d, 16);
	spi->transfer(s, d, 24);
	spi->transfer(s, d, 32);
	spi->transfer(s, d, 48);
	spi->transfer(s, d, sizeof(s));
	printf("Autual Frequncy:%ld", spi->get_actual_frequency());

	PROBE(1, 1);

	for (uint32_t i = 0; i < sizeof(s); i++) {
		s[i] = i;
		d[i] = 0;
	}

}
