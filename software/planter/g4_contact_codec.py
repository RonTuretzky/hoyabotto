"""Lossless float64 contact storage; raw capture/physics remain unchanged."""
import numpy as np

CONTACT_CODEC='g4-contact-f64le-zlib-v1'

def pack_contact_lists(row):
    """Lossless IEEE754 contact encoding; no samples, coordinates or loads removed."""
    import base64,zlib
    packed=dict(row);packed['contact_codec']=CONTACT_CODEC
    for key in ('contacts','step_contacts'):
        values=np.empty((len(row[key]),21),dtype='<f8')
        for i,c in enumerate(row[key]):
            values[i]=[c['geom1'],c['geom2'],*c['position_m'],*np.asarray(c['frame']).ravel(),c['distance_m'],*c['wrench']]
        packed[key]=dict(shape=list(values.shape),zlib_base64=base64.b64encode(zlib.compress(values.tobytes(),1)).decode())
    return packed

def unpack_contact_lists(row):
    import base64,zlib
    if 'contact_codec' not in row:return row
    if row['contact_codec']!=CONTACT_CODEC:raise ValueError('Unrecognized contact codec')
    result=dict(row);result.pop('contact_codec')
    for key in ('contacts','step_contacts'):
        shape=row[key]['shape']
        if not isinstance(shape,list) or len(shape)!=2 or any(type(v) is not int for v in shape) or shape[1]!=21 or not 0<=shape[0]<=1000000:raise ValueError('Invalid contact array dimensions')
        expected=shape[0]*21*8;decoder=zlib.decompressobj()
        raw=decoder.decompress(base64.b64decode(row[key]['zlib_base64'],validate=True),expected+1)
        if len(raw)!=expected or not decoder.eof or decoder.unused_data or decoder.unconsumed_tail:raise ValueError('Contact payload byte count or compressed stream mismatch')
        values=np.frombuffer(raw,dtype='<f8').reshape(shape)
        if not np.isfinite(values).all():raise ValueError('Nonfinite contact array')
        contacts=[]
        for v in values:
            if any(x<0 or x!=int(x) for x in v[:2]):raise ValueError('Nonintegral geom identifier')
            contacts.append(dict(geom1=int(v[0]),geom2=int(v[1]),position_m=v[2:5].tolist(),frame=v[5:14].reshape(3,3).tolist(),distance_m=float(v[14]),wrench=v[15:21].tolist()))
        result[key]=contacts
    return result


# Explicit public scorer-boundary alias; input/output is the complete state row.
decode_contacts=unpack_contact_lists
