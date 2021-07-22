#ifndef _AUDIOCONF_H
#define _AUDIOCONF_H

#define ANDROIDBOOT_AUDIOSYSTEM_CONF 2
/* get_info_audiosystem_conf returns
 * 0 for Internal AMP Processing,
 * 1 for External AMP Processing and
 * -1 for Unknown
 */
char *get_info_audiosystem_conf(void);

#endif /* _AUDIOCONF_H */
