// THIS IS AN AUTOMATICALLY GENERATED FILE.  DO NOT MODIFY
// BY HAND!!
//
// Generated by lcm-gen

#include <string.h>
#include "lcmtypes/sampling_sample_draw.h"

static int __sampling_sample_draw_hash_computed;
static uint64_t __sampling_sample_draw_hash;

uint64_t __sampling_sample_draw_hash_recursive(const __lcm_hash_ptr *p)
{
    const __lcm_hash_ptr *fp;
    for (fp = p; fp != NULL; fp = fp->parent)
        if (fp->v == __sampling_sample_draw_get_hash)
            return 0;

    __lcm_hash_ptr cp;
    cp.parent =  p;
    cp.v = __sampling_sample_draw_get_hash;
    (void) cp;

    uint64_t hash = (uint64_t)0x353c2e8252453c2cLL
         + __boolean_hash_recursive(&cp)
        ;

    return (hash<<1) + ((hash>>63)&1);
}

int64_t __sampling_sample_draw_get_hash(void)
{
    if (!__sampling_sample_draw_hash_computed) {
        __sampling_sample_draw_hash = (int64_t)__sampling_sample_draw_hash_recursive(NULL);
        __sampling_sample_draw_hash_computed = 1;
    }

    return __sampling_sample_draw_hash;
}

int __sampling_sample_draw_encode_array(void *buf, int offset, int maxlen, const sampling_sample_draw *p, int elements)
{
    int pos = 0, element;
    int thislen;

    for (element = 0; element < elements; element++) {

        thislen = __boolean_encode_array(buf, offset + pos, maxlen - pos, &(p[element].if_draw), 1);
        if (thislen < 0) return thislen; else pos += thislen;

    }
    return pos;
}

int sampling_sample_draw_encode(void *buf, int offset, int maxlen, const sampling_sample_draw *p)
{
    int pos = 0, thislen;
    int64_t hash = __sampling_sample_draw_get_hash();

    thislen = __int64_t_encode_array(buf, offset + pos, maxlen - pos, &hash, 1);
    if (thislen < 0) return thislen; else pos += thislen;

    thislen = __sampling_sample_draw_encode_array(buf, offset + pos, maxlen - pos, p, 1);
    if (thislen < 0) return thislen; else pos += thislen;

    return pos;
}

int __sampling_sample_draw_encoded_array_size(const sampling_sample_draw *p, int elements)
{
    int size = 0, element;
    for (element = 0; element < elements; element++) {

        size += __boolean_encoded_array_size(&(p[element].if_draw), 1);

    }
    return size;
}

int sampling_sample_draw_encoded_size(const sampling_sample_draw *p)
{
    return 8 + __sampling_sample_draw_encoded_array_size(p, 1);
}

int __sampling_sample_draw_decode_array(const void *buf, int offset, int maxlen, sampling_sample_draw *p, int elements)
{
    int pos = 0, thislen, element;

    for (element = 0; element < elements; element++) {

        thislen = __boolean_decode_array(buf, offset + pos, maxlen - pos, &(p[element].if_draw), 1);
        if (thislen < 0) return thislen; else pos += thislen;

    }
    return pos;
}

int __sampling_sample_draw_decode_array_cleanup(sampling_sample_draw *p, int elements)
{
    int element;
    for (element = 0; element < elements; element++) {

        __boolean_decode_array_cleanup(&(p[element].if_draw), 1);

    }
    return 0;
}

int sampling_sample_draw_decode(const void *buf, int offset, int maxlen, sampling_sample_draw *p)
{
    int pos = 0, thislen;
    int64_t hash = __sampling_sample_draw_get_hash();

    int64_t this_hash;
    thislen = __int64_t_decode_array(buf, offset + pos, maxlen - pos, &this_hash, 1);
    if (thislen < 0) return thislen; else pos += thislen;
    if (this_hash != hash) return -1;

    thislen = __sampling_sample_draw_decode_array(buf, offset + pos, maxlen - pos, p, 1);
    if (thislen < 0) return thislen; else pos += thislen;

    return pos;
}

int sampling_sample_draw_decode_cleanup(sampling_sample_draw *p)
{
    return __sampling_sample_draw_decode_array_cleanup(p, 1);
}

int __sampling_sample_draw_clone_array(const sampling_sample_draw *p, sampling_sample_draw *q, int elements)
{
    int element;
    for (element = 0; element < elements; element++) {

        __boolean_clone_array(&(p[element].if_draw), &(q[element].if_draw), 1);

    }
    return 0;
}

sampling_sample_draw *sampling_sample_draw_copy(const sampling_sample_draw *p)
{
    sampling_sample_draw *q = (sampling_sample_draw*) malloc(sizeof(sampling_sample_draw));
    __sampling_sample_draw_clone_array(p, q, 1);
    return q;
}

void sampling_sample_draw_destroy(sampling_sample_draw *p)
{
    __sampling_sample_draw_decode_array_cleanup(p, 1);
    free(p);
}

int sampling_sample_draw_publish(lcm_t *lc, const char *channel, const sampling_sample_draw *p)
{
      int max_data_size = sampling_sample_draw_encoded_size (p);
      uint8_t *buf = (uint8_t*) malloc (max_data_size);
      if (!buf) return -1;
      int data_size = sampling_sample_draw_encode (buf, 0, max_data_size, p);
      if (data_size < 0) {
          free (buf);
          return data_size;
      }
      int status = lcm_publish (lc, channel, buf, data_size);
      free (buf);
      return status;
}

struct _sampling_sample_draw_subscription_t {
    sampling_sample_draw_handler_t user_handler;
    void *userdata;
    lcm_subscription_t *lc_h;
};
static
void sampling_sample_draw_handler_stub (const lcm_recv_buf_t *rbuf,
                            const char *channel, void *userdata)
{
    int status;
    sampling_sample_draw p;
    memset(&p, 0, sizeof(sampling_sample_draw));
    status = sampling_sample_draw_decode (rbuf->data, 0, rbuf->data_size, &p);
    if (status < 0) {
        fprintf (stderr, "error %d decoding sampling_sample_draw!!!\n", status);
        return;
    }

    sampling_sample_draw_subscription_t *h = (sampling_sample_draw_subscription_t*) userdata;
    h->user_handler (rbuf, channel, &p, h->userdata);

    sampling_sample_draw_decode_cleanup (&p);
}

sampling_sample_draw_subscription_t* sampling_sample_draw_subscribe (lcm_t *lcm,
                    const char *channel,
                    sampling_sample_draw_handler_t f, void *userdata)
{
    sampling_sample_draw_subscription_t *n = (sampling_sample_draw_subscription_t*)
                       malloc(sizeof(sampling_sample_draw_subscription_t));
    n->user_handler = f;
    n->userdata = userdata;
    n->lc_h = lcm_subscribe (lcm, channel,
                                 sampling_sample_draw_handler_stub, n);
    if (n->lc_h == NULL) {
        fprintf (stderr,"couldn't reg sampling_sample_draw LCM handler!\n");
        free (n);
        return NULL;
    }
    return n;
}

int sampling_sample_draw_subscription_set_queue_capacity (sampling_sample_draw_subscription_t* subs,
                              int num_messages)
{
    return lcm_subscription_set_queue_capacity (subs->lc_h, num_messages);
}

int sampling_sample_draw_unsubscribe(lcm_t *lcm, sampling_sample_draw_subscription_t* hid)
{
    int status = lcm_unsubscribe (lcm, hid->lc_h);
    if (0 != status) {
        fprintf(stderr,
           "couldn't unsubscribe sampling_sample_draw_handler %p!\n", hid);
        return -1;
    }
    free (hid);
    return 0;
}

