#ifndef _MODEL_H_
#define _MODEL_H_

struct Limit {
    u8 flags;
//    u8 safetysw;
//    s16 safetyval;  // allow safetyval to be over +/-125
//    u8 max;
//    u8 min;
//    u8 servoscale;
//    u8 servoscale_neg;
    s8 failsafe;
//    u8 speed;     //measured in degrees/100msec
//    s16 subtrim;  // need to support value greater than 250
};
#define CH_FAILSAFE_EN 0x02
#define NUM_PROTO_OPTS 4
struct Model {
    u32 fixed_id;
//    enum ModelType type;
//	enum Protocols protocol;
    s16 proto_opts[NUM_PROTO_OPTS];
    u8 num_channels;
//    u8 num_ppmin;
//    u16 ppmin_centerpw;
//    u16 ppmin_deltapw;
//    u8 train_sw;
	u8 tx_power;
//    enum SwashType swash_type;
//    u8 swash_invert;
//    u8 swashmix[3];
//    char name[24];
//    char icon[24];
//    char virtname[NUM_VIRT_CHANNELS][VIRT_NAME_LEN];
//    u8 templates[NUM_CHANNELS];
//    u8 safety[NUM_SOURCES+1];
//    u8 telem_alarm[TELEM_NUM_ALARMS];
//    s32 telem_alarm_val[TELEM_NUM_ALARMS];
//    u8 telem_flags;
//    MixerMode mixer_mode;
//    s8 ppm_map[MAX_PPM_IN_CHANNELS];
//    u8 padding_1[2];
//    u32 permanent_timer;
#if HAS_VIDEO
//    u8 videosrc;
//    u8 videoch;
//    s8 video_contrast;
//    s8 video_brightness;
#endif
//    struct Trim trims[NUM_TRIMS];
//    struct Mixer mixers[NUM_MIXERS];
    struct Limit limits[NUM_OUT_CHANNELS];
//    struct Timer timer[NUM_TIMERS];
//    struct PageCfg2 pagecfg2;
#if HAS_DATALOG
//    struct datalog datalog;
#endif
};
extern struct Model Model;



/*struct Model {
    u8 num_channels;
    u8 num_ppmin;
    u8 tx_power;
    u32 fixed_id;
    u16 ppmin_centerpw;
    u16 ppmin_deltapw;
    u8 protocol;
    u8 module;
    s16 proto_opts[NUM_PROTO_OPTS];
};
extern struct model Model;
*/
#endif /*_MODEL_H_*/
