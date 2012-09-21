
#ifndef ANDROID_SENSOR_CONFIGURE_H
#define	ANDROID_SENSOR_CONFIGURE_H

enum sensor_type
{
	AML_SENSOR_TYPE_GRAVITY,
	AML_SENSOR_TYPE_LIGHT,
	AML_SENSOR_TYPE_COMPASS,
	AML_SENSOR_TYPE_NONE
};

struct gsensor_config
{
	int	(*filter)(unsigned int , int);
	float 	LSG;	
};

struct sensor_config
{
	const char *name;
	enum sensor_type type;
	union
	{
		struct gsensor_config gs_config;
	/*	Place holder for future expansion
		struct lightsensor_config ls_config;
		struct compass_config co_config;
	*/
	}config;		
};

const struct sensor_config *get_supported_sensor_cfg();
const struct sensor_config *get_dummy_sensor_cfg(enum sensor_type);
void set_dummy_sensor_name(enum sensor_type s_type, const char *name);
#endif
