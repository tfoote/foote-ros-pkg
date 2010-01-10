import gdata.photos.service
import gdata.media
import gdata.geo

import urllib

username = "tbfoote@gmail.com"

gd_client = gdata.photos.service.PhotosService()
gd_client.email = username
gd_client.password = 'none'
gd_client.source = 'tullys-exampleApp-1'
gd_client.ProgrammaticLogin()


#album = gd_client.InsertAlbum(title='New album', summary='This is an album')

print "All albums:"
albums = gd_client.GetUserFeed(user=username)
for album in albums.entry:
  print 'title: %s, number of photos: %s, id: %s' % (album.title.text,
      album.numphotos.text, album.gphoto_id.text)

print "all photos:"
photos = gd_client.GetFeed(
    '/data/feed/api/user/%s/albumid/%s?kind=photo' % (
        username, album.gphoto_id.text))
for photo in photos.entry:
  print 'Photo title:', photo.title.text

print "Getting recent photos"
photos = gd_client.GetUserFeed(kind='photo', limit='10')
for photo in photos.entry:
  print 'Recently added photo title:', photo.title.text

#print help(photo)
#url =  photo.GetMediaURL()
#urllib.urlretrieve(url, photo.title.text)

#gd_client.Delete(album)

#album_url = '/data/feed/api/user/%s/albumid/%s' % (username, album.gphoto_id.text)
#photo = gd_client.InsertPhotoSimple(album_url, 'New Photo', 
#    'Uploaded using the API', 'DSCN4074.jpg', content_type='image/jpeg')
