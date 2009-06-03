import gdata.photos.service
import gdata.media
import gdata.geo

import sys, os

import urllib

import iptcdata

def check_local_dir(dir_name):
    local_dir = os.path.normpath(dir_name)
    if not os.path.isdir(local_dir):
        print "ERROR: Directory does not exist: %s "%local_dir
        ###\todo make this except
        sys.exit(0)
    else:
        return local_dir

def find_local_files(a_dir):
    a_dir = check_local_dir(a_dir)
    images = []

    for root, dir, files in os.walk(a_dir):
        #print root
        #print dir
        #print files
        for f in files:
            fname = os.path.join(root, f) 
            if fname.startswith(a_dir):
                fname = fname[len(a_dir)+1:] # +1 for /
            #print fname
            basename, ext = os.path.splitext(f)
            if ext == '.jpg':
                images.append(fname)
                #print "appending image %s"%fname
    return images


class WebAlbumUploader:
    def __init__(self, username, password):
        self.username = username
        self.password = password
        self.slash_str = '>'
        self.program_name = 'Tullys WebAlbum Uploader'

        self.setup_google_client()
        


    def setup_google_client(self):
        self.gd_client = gdata.photos.service.PhotosService()
        self.gd_client.email = self.username
        self.gd_client.password = self.password
        self.gd_client.source = self.program_name
        self.gd_client.ProgrammaticLogin()
        
    def find_album(self, album_name, auto_create=False):
        albums = self.gd_client.GetUserFeed(user=self.username)
        for album in albums.entry:
            if album.title.text == album_name:
                return album
        if auto_create:
            return self.gd_client.InsertAlbum(title=self.album_name, summary='Web Album Uploader Album')            



    def get_photo_list_from_server(self, album_url):
        if not self.gd_client:
            return None
        return self.gd_client.GetFeed('%s?kind=photo' % (album_url))

    def upload_tags(self, filename, gphoto):
        iptc_handle = None
        try:
            iptc_handle = iptcdata.open(filename)
        except:
            print "iptcdata.open of %s failed"%filename
        if (len(iptc_handle.datasets) == 0):
            print "No IPTC data!"
            sys.exit(0)

        new_tags = []
        for ds in iptc_handle.datasets:
            #print "Tag: %d, Record: %d" %(ds.record, ds.tag)
            #print "Title: %s"%ds.title
            #descr = iptcdata.get_tag_description(record=ds.record, tag=ds.tag)
            #print "Description: %s" % (descr)
            #print "Value: %s" % (ds.value)
            if ds.title == 'Keywords':
                new_tags.append(ds.value)

        new_tags_str = ', '.join(new_tags)

        if not gphoto.media:
            gphoto.media = gdata.media.Group()
        if not gphoto.media.keywords:
            gphoto.media.keywords = gdata.media.Keywords()

        #if new keywords upload them again
        if not gphoto.media.keywords.text == new_tags_str: ###\todo need to check for title and caption too
            print "Uploading Tags"
            print "Old tags ", gphoto.media.keywords.text
            print "New tags ", new_tags_str
            gphoto.media.keywords.text = new_tags_str
            gphoto = self.gd_client.UpdatePhotoMetadata(gphoto)
        else:
            print "Tags up to date."
        iptc_handle.close()
                    

    def upload(self, album_name, local_dir):
        album_ref = self.find_album(album_name, True)
        album_url = '/data/feed/api/user/%s/albumid/%s' % (username, album_ref.gphoto_id.text)
        for local_filename in find_local_files(local_dir):
            fullfilename = os.path.join(local_dir, local_filename)
            #print "Looking to upload %s"%i
            match_found = False
            for p in self.get_photo_list_from_server(album_url).entry:
                if p.title.text.replace(self.slash_str,'/') == local_filename:
                    #print "found photo %s"%local_filename
                    match_found = True
                    break
                else:
                    #print "%s didn't match %s"%(p.title.text, local_filename)
                    pass
            if  not match_found:
                print "Uploading photo %s"%local_filename
                photo_uploaded = self.gd_client.InsertPhotoSimple(self.album_url, local_filename.replace('/',self.slash_str), 
                                                             'Caption: Uploaded using the API', fullfilename, content_type='image/jpeg')
            else:
                print "Photo %s already uploaded"%fullfilename
            self.upload_tags(fullfilename, p)

    def download(self, album_name, local_dir):
        album_ref = self.find_album(album_name, True)
        album_url = '/data/feed/api/user/%s/albumid/%s' % (username, album_ref.gphoto_id.text)
        for p in self.get_photo_list_from_server(album_url).entry:
            filename = p.title.text.replace(self.slash_str,'/')
            fullname = os.path.join(local_dir, filename)
            if not os.path.exists(os.path.dirname(fullname)):
                os.makedirs(os.path.dirname(fullname))
            if not os.path.exists(fullname):
                urllib.urlretrieve(p.GetMediaURL(), fullname)
                print "Saved %s: "%fullname
            else:
                print "Already Present, Skipped: %s"%fullname
            ###\todo check for size/checksum and date if older replace
            


username = "tbfoote@gmail.com"
password = "tula1984"

album_name = 'test'
local_dir = '/home/tfoote/gdata_test'

uploader = WebAlbumUploader(username, password)
uploader.upload(album_name, local_dir)



#uploader.download(album_name, '/tmp/trial_photo_repo')
