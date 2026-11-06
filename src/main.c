#include <zephyr/logging/log.h>

#include <moth/light_acquisition.h>
#include <moth/phototaxis.h>

LOG_MODULE_REGISTER(moth_app, LOG_LEVEL_INF);

int main(void)
{
	int rc;

	LOG_INF("Moth ADC/light/phototaxis pipeline starting");

	rc = moth_light_acquisition_start();
	if (rc != 0) {
		LOG_ERR("Could not start light acquisition: %d", rc);
		return rc;
	}

	rc = moth_phototaxis_start();
	if (rc != 0) {
		LOG_ERR("Could not start phototaxis controller: %d", rc);
		return rc;
	}

	return 0;
}
