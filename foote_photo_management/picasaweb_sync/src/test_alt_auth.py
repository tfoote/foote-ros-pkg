import gdata.photos.service
import gdata.media
import gdata.geo

import urllib


def GetAuthSubUrl():
  next = 'http://www.example.com/welcome.pyc'
  scope = 'http://picasaweb.google.com/data/'
  secure = False
  session = True
  gd_client = gdata.photos.service.PhotosService()
  return gd_client.GenerateAuthSubURL(next, scope, secure, session);

authSubUrl = GetAuthSubUrl();
print '<a href="%s">Login to your Google account</a>' % authSubUrl






