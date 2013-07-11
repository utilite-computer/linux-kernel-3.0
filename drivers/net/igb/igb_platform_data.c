#include <linux/module.h>
#include <linux/err.h>
#include <linux/igb.h>

static struct igb_platform_data *platform_data = NULL;

int igb_set_platform_data(const struct igb_platform_data *data)
{
        if (platform_data)
                return -EBUSY;
        if (!data)
                return -EINVAL;

        platform_data = kmemdup(data, sizeof(*data), GFP_KERNEL);
        if (!platform_data)
                return -ENOMEM;

        return 0;
}
EXPORT_SYMBOL(igb_set_platform_data);

struct igb_platform_data *igb_get_platform_data(void)
{
	if (!platform_data)
		return ERR_PTR(-ENODEV);

	return platform_data;
}
EXPORT_SYMBOL(igb_get_platform_data);
