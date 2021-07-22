
#define ANDROIDBOOT_CAMERATYPE  10
static char androidboot_cameratype[ANDROIDBOOT_CAMERATYPE];

unsigned char oem_camera_type = 0;
EXPORT_SYMBOL(oem_camera_type);

static int __init get_androidboot_cameratype(char *str)
{
        strlcpy(androidboot_cameratype, str, ANDROIDBOOT_CAMERATYPE);
        return 1;
}
__setup("androidboot.cameratype=", get_androidboot_cameratype);

void camera_init(void)
{
        if(!strcmp(androidboot_cameratype, "dg")){
                oem_camera_type = 1; // 1280x800
        }
        else if(!strcmp(androidboot_cameratype, "dg2")){
                oem_camera_type = 2; // 1024x768
        }
}
