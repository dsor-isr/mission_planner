#include <linux/build-salt.h>
#include <linux/module.h>
#include <linux/vermagic.h>
#include <linux/compiler.h>

BUILD_SALT;

MODULE_INFO(vermagic, VERMAGIC_STRING);
MODULE_INFO(name, KBUILD_MODNAME);

__visible struct module __this_module
__section(.gnu.linkonce.this_module) = {
	.name = KBUILD_MODNAME,
	.init = init_module,
#ifdef CONFIG_MODULE_UNLOAD
	.exit = cleanup_module,
#endif
	.arch = MODULE_ARCH_INIT,
};

#ifdef CONFIG_RETPOLINE
MODULE_INFO(retpoline, "Y");
#endif

static const struct modversion_info ____versions[]
__used __section(__versions) = {
	{ 0xb3753869, "module_layout" },
	{ 0xf7744a0f, "pci_unregister_driver" },
	{ 0xc38c6801, "device_destroy" },
	{ 0x2b9ca1ff, "__pci_register_driver" },
	{ 0x6091b333, "unregister_chrdev_region" },
	{ 0x78f44845, "cdev_del" },
	{ 0xb356c301, "class_destroy" },
	{ 0xc79de84d, "device_create" },
	{ 0x8d62ea07, "__class_create" },
	{ 0x7afe113a, "cdev_add" },
	{ 0xa3036ef8, "cdev_init" },
	{ 0xe3ec2f2b, "alloc_chrdev_region" },
	{ 0xdecd0b29, "__stack_chk_fail" },
	{ 0xb44ad4b3, "_copy_to_user" },
	{ 0x362ef408, "_copy_from_user" },
	{ 0x23de47c2, "pci_release_regions" },
	{ 0x8fae8099, "pci_disable_device" },
	{ 0x3bb68935, "pci_request_regions" },
	{ 0xc089e356, "pci_enable_device" },
	{ 0xc5850110, "printk" },
	{ 0xbdfb6dbb, "__fentry__" },
};

MODULE_INFO(depends, "");


MODULE_INFO(srcversion, "1EB0B7F35699348DBF2B37A");
