#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(moth_app, LOG_LEVEL_INF);

int main(void)
{
	LOG_INF("Moth app skeleton starting");
	return 0;
}
