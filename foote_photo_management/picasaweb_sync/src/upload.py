#!/usr/bin/env python

import gdata.photos.service
import gdata.media
#import gdata.geo

import sys, os
from optparse import OptionParser
import yaml


import urllib
import iptcdata
import getpass

def check_for_dataset(f, rs):
    for ds in f.datasets:
        if (ds.record, ds.tag) == rs:
            return ds
    return None

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
        
    def list_album_names(self, album_username):
        albums = self.gd_client.GetUserFeed(user=album_username)
        return set([a.title.text for a in albums.entry])

    def find_album(self, album_name, album_username, auto_create=False):
        albums = self.gd_client.GetUserFeed(user=album_username)
        for album in albums.entry:
            if album.title.text == album_name:
                return album
        if auto_create:
            return self.gd_client.InsertAlbum(title=album_name, access="private")            
            #return self.gd_client.InsertAlbum(title=album_name, access="private", summary='Web Album Uploader Album')            



    def get_photo_list_from_server(self, album_url):
        if not self.gd_client:
            return None
        return self.gd_client.GetFeed('%s?kind=photo' % (album_url))

    def upload_tags(self, filename, gphoto):
        try:
            iptc_handle = iptcdata.open(filename)
        except:
            print "iptcdata failed to open %s"%filename
            return


        if (len(iptc_handle.datasets) == 0):
            print "No IPTC data!"
            #sys.exit(0)

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

    def download_tags(self, filename, gphoto):
        print "downloading tags for ", filename
        try:
            iptc_handle = iptcdata.open(filename)
            print "Opented iptchandle", filename
        except:
            print "iptcdata open failed on %s"%filename
            return

        if (len(iptc_handle.datasets) == 0):
            print "No IPTC data!"
            #sys.exit(0)

        if not gphoto.media:
            gphoto.media = gdata.media.Group()
        if not gphoto.media.keywords:
            gphoto.media.keywords = gdata.media.Keywords()

        if gphoto.media.keywords.text:
            web_tags = set(gphoto.media.keywords.text.split(', '))
        else:
            web_tags = set() # this could return, but we need to save and close

        old_tags = set()
        for ds in iptc_handle.datasets:
            #print "Tag: %d, Record: %d" %(ds.record, ds.tag)
            #print "Title: %s"%ds.title
            #descr = iptcdata.get_tag_description(record=ds.record, tag=ds.tag)
            #print "Description: %s" % (descr)
            #print "Value: %s" % (ds.value)
            if ds.title == 'Keywords':
                if ds.value in web_tags:
                    old_tags.add(ds.value)
        new_tags = web_tags - old_tags

        rs = iptcdata.find_record_by_name("Keywords")
        for tag in new_tags:
            ds = iptc_handle.add_dataset(rs)
            ds.value = tag
            print "adding tag %s"%(tag)

        try:
            iptc_handle.save()
            iptc_handle.close()
            print "Closed iptc handle %s"%filename
        except:
            print "iptcdata save or close failed on %s"%filename
                    
                    

    def upload(self, album_name, local_dir, album_user=False):
        if not album_user:
            album_user = self.username
        album_ref = self.find_album(album_name, album_user, True)
        if not album_ref:
            print >> sys.stderr, "No album found into which to upload."
            return False
        album_url = '/data/feed/api/user/%s/albumid/%s' % (album_user, album_ref.gphoto_id.text)
        for local_filename in find_local_files(os.path.join(local_dir, album_name)):
            fullfilename = os.path.join(local_dir, album_name, local_filename)
            #print "Looking to upload %s"%i
            match_found = False
            photo = None
            for p in self.get_photo_list_from_server(album_url).entry:
                if p.title.text.replace(self.slash_str,'/') == local_filename:
                    #print "found photo %s"%local_filename
                    photo = p
                    break
                else:
                    #print "%s didn't match %s"%(p.title.text, local_filename)
                    pass
            if  not photo:
                print "Uploading photo %s"%local_filename
                print fullfilename
                photo = self.gd_client.InsertPhotoSimple(album_url, local_filename.replace('/',self.slash_str), 
                                                         'Caption: Uploaded using the API', fullfilename, content_type='image/jpeg')

            else:
                print "Photo %s already uploaded"%fullfilename
            self.upload_tags(fullfilename, photo)
        return True

    def download(self, album_name, local_dir, album_user=False):
        print "downloading ", album_name
        if not album_user:
            album_user = self.username
        album_ref = self.find_album(album_name, album_user, False)
        if not album_ref:
            
            print "Failed to find album, %s on username %s "%(album_name, album_user)
            return False
        album_url = '/data/feed/api/user/%s/albumid/%s' % (album_user, album_ref.gphoto_id.text)
        for p in self.get_photo_list_from_server(album_url).entry:
            filename = p.title.text.replace(self.slash_str,'/')
            fullname = os.path.join(local_dir, album_name, filename)
            if not os.path.exists(os.path.dirname(fullname)):
                os.makedirs(os.path.dirname(fullname))
            if not os.path.exists(fullname):
                urllib.urlretrieve(p.GetMediaURL(), fullname)
                print "Saved %s: "%fullname
            else:
                print "Already Present, Skipped: %s"%fullname
            self.download_tags(fullname, p)
            ###\todo check for size/checksum and date if older replace
            








if __name__ == '__main__':
    parser = OptionParser(usage="usage: %prog COMMAND FILES", prog='upload')
#    parser.add_option("--username", dest="username", default='',
#                     type="string", help="username to use")
#    parser.add_option("--password", dest="password", default='',
#                      type="string", help="password to use")
#    parser.add_option("--album_name", dest="album_name", default='',
#                      type="string", help="album_name to use")
#    parser.add_option("--local_dir", dest="local_dir", default='.',
#                      type="string", help="local_dir to use")

    options, args = parser.parse_args()

    possible_cmds = ["upload", "download", "downloadall"]

    if len(args) < 1:
        parser.error("Please enter a command")

    cmd = args[0]
    if cmd not in possible_cmds:
        parser.error("Command must be either 'upload' or 'download'")

    if len(args) < 2:
        parser.error("Command %s requires additional arguments of yaml files"%cmd)
    
    yaml_files = args[1:]

    for f in yaml_files:
        if not os.path.exists(f):
            parser.error("File: %s doesn't exits."%f)
        with open(f) as fh:
            file_text = fh.read()
            try:
                file_map = yaml.load(file_text)
            except yaml.YAMLError, exc:
                parser.error("Failed parsing yaml while processing %s\n"%path, exc)
                
        #print "yaml result is ", file_map
        required_fields = set(["username", "album", "album_username", "local_path"])
        missing_fields = required_fields - set(file_map.keys())
        extra_fields = set(file_map.keys()) - required_fields
        #print "missing fields = ", missing_fields
        #print "extra fields = ", extra_fields

        
        password = getpass.getpass("Please enter password for username %s:"%file_map["username"])
        
        uploader = WebAlbumUploader(file_map["username"], password)
        if cmd == "upload":
            uploader.upload(file_map["album"], file_map["local_path"], file_map["album_username"])
        if cmd == "download":
            uploader.download(file_map["album"], file_map["local_path"], file_map["album_username"])
        if cmd == "downloadall":
            albums = uploader.list_album_names(file_map["album_username"])
            for a in albums:
                print "album name", a, " of ",albums 
                uploader.download(a, file_map["local_path"], file_map["album_username"])
                
