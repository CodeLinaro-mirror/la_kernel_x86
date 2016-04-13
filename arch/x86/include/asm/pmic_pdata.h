#ifndef __PMIC_PDATA_H__
#define __PMIC_PDATA_H__

struct temp_lookup {
	int adc_val;
	int temp;
	int temp_err;
};

struct notifier_block;

/*
 * pmic cove charger driver info
 */
struct pmic_platform_data {
	void (*cc_to_reg)(int, u8*);
	void (*cv_to_reg)(int, u8*);
	void (*inlmt_to_reg)(int, u8*);
	void (*notify_charging_stat)(bool) __deprecated;
	int max_tbl_row_cnt;
	struct temp_lookup *adc_tbl;
};

extern int pmic_get_status(void);
extern int pmic_enable_charging(bool);
extern int pmic_get_ext_charging_status(bool *is_charging);
extern int pmic_set_cc(int);
extern int pmic_set_cv(int);
extern int pmic_set_ilimma(int);
extern int pmic_enable_vbus(bool enable);
extern int pmic_handle_otgmode(bool enable);
/* WA for ShadyCove VBUS removal detect issue */
extern int pmic_handle_low_supply(void);

extern void dump_pmic_regs(void);
#ifdef CONFIG_PMIC_CCSM
extern int pmic_get_health(void);
extern int pmic_get_battery_pack_temp(int *);
#else
static int pmic_get_health(void)
{
	return 0;
}
static int pmic_get_battery_pack_temp(int *temp)
{
	return 0;
}
#endif

enum pmic_notifier_action {
	PMIC_ACTION_BATTERY_ZONE_CHANGED,
	PMIC_ACTION_OVERHEAT,
	PMIC_ACTION_CHARGING_STATUS,
	PMIC_ACTION_MAX
};

extern int register_pmic_notifier(struct notifier_block *nb);
extern void unregister_pmic_notifier(struct notifier_block *nb);

#endif
