// THIS IS AN AUTOMATICALLY GENERATED FILE.  DO NOT MODIFY
// BY HAND!!
//
// Generated by lcm-gen

#include <string.h>
#include "lcmtypes/sampling_workspace_size_data.h"

static int __sampling_workspace_size_data_hash_computed;
static uint64_t __sampling_workspace_size_data_hash;

uint64_t __sampling_workspace_size_data_hash_recursive(const __lcm_hash_ptr *p)
{
    const __lcm_hash_ptr *fp;
    for (fp = p; fp != NULL; fp = fp->parent)
        if (fp->v == __sampling_workspace_size_data_get_hash)
            return 0;

    __lcm_hash_ptr cp;
    cp.parent =  p;
    cp.v = (void*)__sampling_workspace_size_data_get_hash;
    (void) cp;

    uint64_t hash = (uint64_t)0x8e486f48789558bcLL
         + __double_hash_recursive(&cp)
         + __double_hash_recursive(&cp)
        ;

    return (hash<<1) + ((hash>>63)&1);
}

int64_t __sampling_workspace_size_data_get_hash(void)
{
    if (!__sampling_workspace_size_data_hash_computed) {
        __sampling_workspace_size_data_hash = (int64_t)__sampling_workspace_size_data_hash_recursive(NULL);
        __sampling_workspace_size_data_hash_computed = 1;
    }

    return __sampling_workspace_size_data_hash;
}

int __sampling_workspace_size_data_encode_array(void *buf, int offset, int maxlen, const sampling_workspace_size_data *p, int elements)
{
    int pos = 0, element;
    int thislen;

    for (element = 0; element < elements; element++) {

        thislen = __double_encode_array(buf, offset + pos, maxlen - pos, &(p[element].size_x), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __double_encode_array(buf, offset + pos, maxlen - pos, &(p[element].size_y), 1);
        if (thislen < 0) return thislen; else pos += thislen;

    }
    return pos;
}

int sampling_workspace_size_data_encode(void *buf, int offset, int maxlen, const sampling_workspace_size_data *p)
{
    int pos = 0, thislen;
    int64_t hash = __sampling_workspace_size_data_get_hash();

    thislen = __int64_t_encode_array(buf, offset + pos, maxlen - pos, &hash, 1);
    if (thislen < 0) return thislen; else pos += thislen;

    thislen = __sampling_workspace_size_data_encode_array(buf, offset + pos, maxlen - pos, p, 1);
    if (thislen < 0) return thislen; else pos += thislen;

    return pos;
}

int __sampling_workspace_size_data_encoded_array_size(const sampling_workspace_size_data *p, int elements)
{
    int size = 0, element;
    for (element = 0; element < elements; element++) {

        size += __double_encoded_array_size(&(p[element].size_x), 1);

        size += __double_encoded_array_size(&(p[element].size_y), 1);

    }
    return size;
}

int sampling_workspace_size_data_encoded_size(const sampling_workspace_size_data *p)
{
    return 8 + __sampling_workspace_size_data_encoded_array_size(p, 1);
}

int __sampling_workspace_size_data_decode_array(const void *buf, int offset, int maxlen, sampling_workspace_size_data *p, int elements)
{
    int pos = 0, thislen, element;

    for (element = 0; element < elements; element++) {

        thislen = __double_decode_array(buf, offset + pos, maxlen - pos, &(p[element].size_x), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __double_decode_array(buf, offset + pos, maxlen - pos, &(p[element].size_y), 1);
        if (thislen < 0) return thislen; else pos += thislen;

    }
    return pos;
}

int __sampling_workspace_size_data_decode_array_cleanup(sampling_workspace_size_data *p, int elements)
{
    int element;
    for (element = 0; element < elements; element++) {

        __double_decode_array_cleanup(&(p[element].size_x), 1);

        __double_decode_array_cleanup(&(p[element].size_y), 1);

    }
    return 0;
}

int sampling_workspace_size_data_decode(const void *buf, int offset, int maxlen, sampling_workspace_size_data *p)
{
    int pos = 0, thislen;
    int64_t hash = __sampling_workspace_size_data_get_hash();

    int64_t this_hash;
    thislen = __int64_t_decode_array(buf, offset + pos, maxlen - pos, &this_hash, 1);
    if (thislen < 0) return thislen; else pos += thislen;
    if (this_hash != hash) return -1;

    thislen = __sampling_workspace_size_data_decode_array(buf, offset + pos, maxlen - pos, p, 1);
    if (thislen < 0) return thislen; else pos += thislen;

    return pos;
}

int sampling_workspace_size_data_decode_cleanup(sampling_workspace_size_data *p)
{
    return __sampling_workspace_size_data_decode_array_cleanup(p, 1);
}

int __sampling_workspace_size_data_clone_array(const sampling_workspace_size_data *p, sampling_workspace_size_data *q, int elements)
{
    int element;
    for (element = 0; element < elements; element++) {

        __double_clone_array(&(p[element].size_x), &(q[element].size_x), 1);

        __double_clone_array(&(p[element].size_y), &(q[element].size_y), 1);

    }
    return 0;
}

sampling_workspace_size_data *sampling_workspace_size_data_copy(const sampling_workspace_size_data *p)
{
    sampling_workspace_size_data *q = (sampling_workspace_size_data*) malloc(sizeof(sampling_workspace_size_data));
    __sampling_workspace_size_data_clone_array(p, q, 1);
    return q;
}

void sampling_workspace_size_data_destroy(sampling_workspace_size_data *p)
{
    __sampling_workspace_size_data_decode_array_cleanup(p, 1);
    free(p);
}

int sampling_workspace_size_data_publish(lcm_t *lc, const char *channel, const sampling_workspace_size_data *p)
{
      int max_data_size = sampling_workspace_size_data_encoded_size (p);
      uint8_t *buf = (uint8_t*) malloc (max_data_size);
      if (!buf) return -1;
      int data_size = sampling_workspace_size_data_encode (buf, 0, max_data_size, p);
      if (data_size < 0) {
          free (buf);
          return data_size;
      }
      int status = lcm_publish (lc, channel, buf, data_size);
      free (buf);
      return status;
}

struct _sampling_workspace_size_data_subscription_t {
    sampling_workspace_size_data_handler_t user_handler;
    void *userdata;
    lcm_subscription_t *lc_h;
};
static
void sampling_workspace_size_data_handler_stub (const lcm_recv_buf_t *rbuf,
                            const char *channel, void *userdata)
{
    int status;
    sampling_workspace_size_data p;
    memset(&p, 0, sizeof(sampling_workspace_size_data));
    status = sampling_workspace_size_data_decode (rbuf->data, 0, rbuf->data_size, &p);
    if (status < 0) {
        fprintf (stderr, "error %d decoding sampling_workspace_size_data!!!\n", status);
        return;
    }

    sampling_workspace_size_data_subscription_t *h = (sampling_workspace_size_data_subscription_t*) userdata;
    h->user_handler (rbuf, channel, &p, h->userdata);

    sampling_workspace_size_data_decode_cleanup (&p);
}

sampling_workspace_size_data_subscription_t* sampling_workspace_size_data_subscribe (lcm_t *lcm,
                    const char *channel,
                    sampling_workspace_size_data_handler_t f, void *userdata)
{
    sampling_workspace_size_data_subscription_t *n = (sampling_workspace_size_data_subscription_t*)
                       malloc(sizeof(sampling_workspace_size_data_subscription_t));
    n->user_handler = f;
    n->userdata = userdata;
    n->lc_h = lcm_subscribe (lcm, channel,
                                 sampling_workspace_size_data_handler_stub, n);
    if (n->lc_h == NULL) {
        fprintf (stderr,"couldn't reg sampling_workspace_size_data LCM handler!\n");
        free (n);
        return NULL;
    }
    return n;
}

int sampling_workspace_size_data_subscription_set_queue_capacity (sampling_workspace_size_data_subscription_t* subs,
                              int num_messages)
{
    return lcm_subscription_set_queue_capacity (subs->lc_h, num_messages);
}

int sampling_workspace_size_data_unsubscribe(lcm_t *lcm, sampling_workspace_size_data_subscription_t* hid)
{
    int status = lcm_unsubscribe (lcm, hid->lc_h);
    if (0 != status) {
        fprintf(stderr,
           "couldn't unsubscribe sampling_workspace_size_data_handler %p!\n", hid);
        return -1;
    }
    free (hid);
    return 0;
}

