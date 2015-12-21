#include <linux/module.h>
#include <linux/vermagic.h>
#include <linux/compiler.h>

MODULE_INFO(vermagic, VERMAGIC_STRING);

__visible struct module __this_module
__attribute__((section(".gnu.linkonce.this_module"))) = {
	.name = KBUILD_MODNAME,
	.init = init_module,
#ifdef CONFIG_MODULE_UNLOAD
	.exit = cleanup_module,
#endif
	.arch = MODULE_ARCH_INIT,
};

static const struct modversion_info ____versions[]
__used
__attribute__((section("__versions"))) = {
	{ 0x31d8dd22, __VMLINUX_SYMBOL_STR(module_layout) },
	{ 0xc3e22d46, __VMLINUX_SYMBOL_STR(platform_driver_unregister) },
	{ 0xb7d82577, __VMLINUX_SYMBOL_STR(__platform_driver_register) },
	{ 0x62ab4085, __VMLINUX_SYMBOL_STR(dev_set_drvdata) },
	{ 0xadaec9e7, __VMLINUX_SYMBOL_STR(sysfs_create_group) },
	{ 0x3d7c8ff, __VMLINUX_SYMBOL_STR(devm_regulator_bulk_get) },
	{ 0x189a70d0, __VMLINUX_SYMBOL_STR(__mutex_init) },
	{ 0xeb6dbc53, __VMLINUX_SYMBOL_STR(devm_kmalloc) },
	{ 0x796c7dbb, __VMLINUX_SYMBOL_STR(regulator_bulk_enable) },
	{ 0x222e7ce2, __VMLINUX_SYMBOL_STR(sysfs_streq) },
	{ 0xd85afe38, __VMLINUX_SYMBOL_STR(regulator_get_voltage) },
	{ 0x91715312, __VMLINUX_SYMBOL_STR(sprintf) },
	{ 0x514c989c, __VMLINUX_SYMBOL_STR(regulator_get_current_limit) },
	{ 0xb6b0e312, __VMLINUX_SYMBOL_STR(regulator_is_enabled) },
	{ 0xb2026c56, __VMLINUX_SYMBOL_STR(dev_err) },
	{ 0x1bb99ecb, __VMLINUX_SYMBOL_STR(mutex_unlock) },
	{ 0x8d1c4ad4, __VMLINUX_SYMBOL_STR(regulator_set_voltage) },
	{ 0x3a6976b9, __VMLINUX_SYMBOL_STR(mutex_lock) },
	{ 0x78780155, __VMLINUX_SYMBOL_STR(regulator_list_voltage) },
	{ 0x11a13e31, __VMLINUX_SYMBOL_STR(_kstrtol) },
	{ 0x379dd40e, __VMLINUX_SYMBOL_STR(regulator_bulk_disable) },
	{ 0xb2b7c4e1, __VMLINUX_SYMBOL_STR(sysfs_remove_group) },
	{ 0x237e133a, __VMLINUX_SYMBOL_STR(dev_get_drvdata) },
	{ 0xb1ad28e0, __VMLINUX_SYMBOL_STR(__gnu_mcount_nc) },
};

static const char __module_depends[]
__used
__attribute__((section(".modinfo"))) =
"depends=";

