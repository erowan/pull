import datetime
import logging
import os
import socket
import sys
import tempfile
from asyncore import compact_traceback
import urllib2
import cookielib
import codecs 
try:
    from selenium.webdriver.remote import webdriver as remote_webdriver
    from selenium.webdriver.chrome import webdriver
    import selenium.webdriver.support.ui as ui
    from selenium.webdriver.support.wait import POLL_FREQUENCY
    from selenium.webdriver.common.desired_capabilities import DesiredCapabilities
    selenium_installed=True
except ImportError:
    selenium_installed=False

__version__ = 0.2
log = logging.getLogger('pull')

def write_cache_file(data, file_path):
    def get_parent_dir(f):
        parentDir, _ = os.path.split(f)
        if parentDir == "":
            raise IOError("no parent directory found for file " + f)
        return parentDir
    # if file dir path does not exist create it
    parent_dir = get_parent_dir(file_path)
    if os.path.isfile(parent_dir):
        raise AssertionError('Expected a cache directory here not a regular'
          ' file %s' % parent_dir)
    if not os.path.exists(parent_dir):
        os.makedirs(parent_dir)
    f = open(file_path, 'wb')
    f.write(data)
    f.close()
    
def write_cache_file2(data, file_path, encoding='utf-8'):
    def get_parent_dir(f):
        parentDir, _ = os.path.split(f)
        if parentDir == "":
            raise IOError("no parent directory found for file " + f)
        return parentDir
    # if file dir path does not exist create it
    parent_dir = get_parent_dir(file_path)
    if os.path.isfile(parent_dir):
        raise AssertionError('Expected a cache directory here not a regular'
          ' file %s' % parent_dir)
    if not os.path.exists(parent_dir):
        os.makedirs(parent_dir)
     
    with codecs.open(file_path, mode='wb', encoding=encoding) as f:    
        f.write(data)
 
class ErrorForAllFeeds(Exception):
    pass

class ErrorForSomeFeeds(Exception):
    pass

class ErrorForAllRequests(Exception):
    pass

class Criteria(object):

    def build(self, start, end):
        pass

class FileListCriteria(Criteria):
    
    temp_dir = tempfile.gettempdir() + '/cache/'
    
    def __init__(self, cache_location=None):
        self.cache_location = cache_location or FileListCriteria.temp_dir
        
    def build(self, start, end):
        pass

class SearchCriteria(Criteria):
    def build(self, start, end):
        pass

class Protocol(object):
    """
    The :term:`Fetch` step is defined by specifying a :class:`Criteria` like 
    :class:`FileListCriteria` to a :class:`Protocol` like :class:`UrlProtocol`.      
    """
    def __init__(self, criteria=None):
        self.criteria = criteria

    def call(self, start, end):
        return self.fetch(self.criteria.build(start, end))

    def fetch(self, input_criteria):
        """
        Fetch the files based on an input criteria. Input is typically a list 
        of url & cache filename tuples returned from a FileListCriteria instance 
        but could be any input suitable for a protocol instance to handle.
        """
        return [input_criteria]

"""
Use :class:`SkipProtocol` to bypass the Protocol step.
"""
SkipProtocol = Protocol

class SeleniumProtocol(Protocol):
    """
    Use Selenium Browser automation framework to get a page back post js load. 
    Uses the HtmlUnitDriver if remote is sepecified otherwise a local chrome 
    driver.          
    """
    def __init__(self, criteria, remote=None, wait_until_cond=None,
                 timeout=10, poll_frequency=None):
        Protocol.__init__(self, criteria)
        if not selenium_installed:
            err_msg = 'Note you must install selenium if you want to use SeleniumProtocol'
            log.error(err_msg) 
            raise ValueError(err_msg)   
        self.remote = remote 
        self.wait_until_cond = wait_until_cond
        self.timeout = timeout
        self.poll_frequency = poll_frequency or POLL_FREQUENCY
    
    def fetch(self, files):
        '''
        fetch urls using selenium by writing url responses to cache_files
        @param files: list of (url, cache_file) tuples
        @return: cache_file list
        '''
        cache_files = []
        failures = []
        if not files:
            raise ErrorForAllRequests('No input files to fetch!')
        browser = remote_webdriver.WebDriver(self.remote,
            desired_capabilities=DesiredCapabilities.HTMLUNITWITHJS) \
                if self.remote else webdriver.WebDriver()           
                        
        for url, f in files:
            try:
                log.info("Downloading: " + str(url))
                browser.get(url)
                if self.wait_until_cond:
                    log.debug("wait until page load condition is satisifed ...")
                    wait = ui.WebDriverWait(browser, self.timeout,
                                            self.poll_frequency)
                    wait.until(self.wait_until_cond)
                response = browser.page_source
                write_cache_file2(response, f)
                cache_files.append(f)
            except Exception, e:
                error_message = 'Exception: %s, for url: %s' % (e, url)
                log.warn(error_message)
           
        try:
            browser.stop_client()
            browser.quit()
        except Exception, e: 
            log.exception(e)
       
        if len(cache_files) == 0:
            # note only raising an exception if all file retrievals failed
            raise ErrorForAllRequests(u'URL Fetch failed for all requests. '\
              'Errors: {0}'.format(">>>".join(failures)))
        return cache_files
    
class UrlProtocol(Protocol):
    """
    :class:`UrlProtocol` is used for url GET fetching.
    """

    def __init__(self, criteria=None, timeout=None,  httpDebugLevel=0,
                 proxies=None, user_agent=None):
        Protocol.__init__(self, criteria=criteria)
        self.cj = cookielib.CookieJar()
        self.proxyHandler = urllib2.ProxyHandler(proxies) if proxies else None    
        self.httpLogger = urllib2.HTTPHandler(debuglevel=httpDebugLevel)
        if timeout:
            socket.setdefaulttimeout(timeout)
        # best to make out we are a browser
        self.userAgent = user_agent or \
  'Mozilla/4.0 (compatible; MSIE 6.0; Windows NT 5.1; SV1; .NET CLR 1.1.4322)'
        self.httpHeaders = {
            'User-agent' : self.userAgent,
            'Proxy-Connection' : 'Keep-Alive',
            'Accept-Encoding' : 'gzip, deflate',
            'Pragma' : 'no-cache',
            'Cache-Control' : 'no-cache',
            'Connection' : 'Keep-Alive'
        }

    def fetch(self, files):
        '''
        fetch urls by writing url responses to cache_files
        @param files: list of (url, cache_file) tuples
        @return: cache_file list
        '''
        cache_files = []
        failures = []
        if not files:
            raise ErrorForAllRequests('No input files to fetch!')
            
        for url, f in files:
            try:
                log.info("Downloading: " + str(url))
                response, headers = self.fetch_url(url,
                                                   headers=self.httpHeaders)
                log.debug('response headers=%s' % str(headers))
                write_cache_file(response, f)
                cache_files.append(f)
            except Exception, e:
                # todo: think about passing failures back by setting
                # stats[feed]['errors'] because if len(cache_files) > 0
                # this failure is only logged here and not accessable later on.
                error_message = 'Exception: %s, for url: %s' % (str(e), url)
                log.warn(error_message)
                failures.append(error_message)

        if len(cache_files) == 0:
            # note only raising an exception if all file retrievals failed
            raise ErrorForAllRequests(u'URL Fetch failed for all requests. '\
              'Errors: {0}'.format(">>>".join(failures)))
        return cache_files

    def fetch_url(self, url, **request_args):
        try:
            handlers = []
            if self.proxyHandler:
                handlers.append(self.proxyHandler)
            handlers.extend([urllib2.HTTPCookieProcessor(self.cj),
                             self.httpLogger])    
            opener = urllib2.build_opener(*handlers)
            response =  opener.open(urllib2.Request(url, **request_args))
            if response.headers.get('Content-Encoding') == 'gzip':
                return self.unzip(response.read()), response.headers
            else:
                return response.read(), response.headers                    

        except urllib2.HTTPError, e:
            err_msg = 'Could not get file for {0}. Error: {1}'.format(url, e)
             
        except urllib2.URLError, e:
            err_msg = 'Failed to reach server. Exception: ' + str(e)
        except IOError, e:
            err_msg = 'IOError Exception: ' + str(e)
        except socket.error:
            errno, errstr = sys.exc_info()[:2]
            if errno == socket.timeout:
                err_msg = 'Socket timeout getting ' + url + ':' + str(errstr)
            else:
                err_msg = 'Some socket error ' + url + ':' + str(errstr)
        except Exception, e:
            err_msg = 'Exception:' + str(e) + ', for url=' + url
        raise ValueError('Exception during fetch_url, exception=%s' % err_msg)

    def unzip(self, gzip_data):
        import gzip
        from StringIO import StringIO
        log.info('unzipping data before storage')
        compressed_stream = StringIO(gzip_data)
        gzipper = gzip.GzipFile(fileobj=compressed_stream)
        data = gzipper.read()
        #log.debug('unzipped_data=%s' % data)
        gzipper.close()
        compressed_stream.close()
        return data

class Updater(object):
    
    def __init__(self):
        # set later in Feed.__init__
        self.cache_location = None
    
    def __call__(self, data_items):
        pass
    
class StoreItems(Updater):
    
    def __init__(self):
        Updater.__init__(self)
        self.data_items = []
    
    def __call__(self, data_items):
        self.data_items.extend(data_items)    

class Parser(object):
    """
    Parser Interface. Subclass to define the term:`Parse` step.
    """
    def __init__(self):
        self.pull_start_date = None
        self.pull_end_date = None
        self.name = 'Parser'
        
    def parse(self, file_path):
        """
        Parse the contents of the file located at ``file_path``.   
        """
        return file_path
    
    def get_logger(self):
        return logging.getLogger(self.name)

"""
Use :class:`SkipParser` to bypass the Parser step.
"""
SkipParser = Parser

class Feed(object):
    '''
    A Feed is run based on a date range via the 'go' method. It achieves this
    by using a configured Protocol helper to download files specified by a
    Criteria. The downloaded files are stored to a local cache and then passed 
    to a configured Parser to produce a series of dicts. Each distinct fileset
    should have it's own Feed which represents a unit of work for the activites
    described above.  
    '''

    def __init__(self, name, protocol, parser, **kwargs):
        '''
        Feed ctor.      
        @param protocol: protocol used to fetch files
        @param parser: parser used to parse files
        @param kwargs: Optional kwargs keys are:
          'commence_date' - start date of series. file downloads will not try
          and download any files before this date if specified.
          'updater' - callable that takes a list of dicts as input
          'expected_series_count' - Number of series expected in Feed     
        '''
        self.name = name
        self.protocol = protocol
        self.parser = parser
        # set defaults
        self.commence_date = None
        self.expected_series_count = None
        self.relative_cache_path = self.name
        # overwrite state with user supplied args
        self.__dict__.update(kwargs)  
        if 'updater' not in self.__dict__:
            self.updater = StoreItems() 
        self.protocol.criteria.cache_location = os.path.join(
          self.protocol.criteria.cache_location, self.relative_cache_path, '')  

        self.updater.cache_location = self.get_cache_location()
        
    def go(self, start, end):
        '''
        Main function to drive a Feeds download & store.
        @param start: starting datetime.date of a feed
        @param end: end datetime.date of a feed
        '''
        cache_files = self.__fetch(start, end)
        if len(cache_files) == 0:
            raise ErrorForAllRequests('fetch returned zero files')

        return self.parse_files(cache_files)


    def parse_files(self, files):
        '''
        Parse input files into data_item dicts and pass into updater.
        @param files: local file list
        '''
        count = 0
        for data_items in self.__parse(files):
            if data_items:
                count += len(data_items)
                self.updater(data_items) 
        return count   

    def get_logger(self):
        return logging.getLogger(self.name)
  
    def get_cache_location(self):
        return self.protocol.criteria.cache_location

    def __fetch(self, start, end):
        self.clear_cache()
        if self.commence_date != None and \
            start.timetuple() < self.commence_date.timetuple():
            msg = ' '.join(('start date:', str(start), 'falls after this',
                'feeds commence date of', str(start),
                ', re-setting start date to commence date for download.'))
            self.get_logger().warn(msg)
            start = self.commence_date

        self.parser.pull_start_date = start
        self.parser.pull_end_date = end
        return self.protocol.call(start, end)


    def clear_cache(self):
        self.__delete_file_tree(self.get_cache_location())

    def __delete_file_tree(self, rootDir):
        # Delete everything reachable from the directory named in 'top',
        # assuming there are no symbolic links.
        # CAUTION:  This is dangerous!  For example, if top == '/', it
        # could delete all your disk files.
        for root, dirs, files in os.walk(rootDir, topdown=False):
            for name in files:
                self.get_logger().debug(name + " File Removed!")
                os.remove(os.path.join(root, name))
            for name in dirs:
                self.get_logger().debug(name + " Dir Removed!")
                os.rmdir(os.path.join(root, name))

    def __delete_files_from_dir(self, cache_loc):
        regularFiles = [f for f in os.listdir(cache_loc) \
                        if os.path.isfile(cache_loc + f)]
        for f in regularFiles:
            os.remove(cache_loc + f)
            self.get_logger().debug(f + " Removed!")    

    def __parse(self, cache_files):
        parser = self.parser
        for file_path in cache_files:
            try:
                self.get_logger().info('Parsing file %s' % file_path)
                yield parser.parse(file_path)
            except Exception, e:
                _, t, v, tbinfo = compact_traceback()
                msg = 'error parsing in {0}:, error={1}, traceback is: ' \
                     '({2}: {3} {4})'.format(file_path, e, t, v, tbinfo)
                self.get_logger().warn(msg) 
             
def build_feed(name, protocol, parser=None, **kwargs):
    parser = parser or SkipParser()
    return Feed(name, protocol, parser, **kwargs)

def go(name, feeds, start, end=None, logLevel=logging.INFO):  
    """
    The main entry point to run feeds.
    
    'feeds' is a list of :class:`Feed` objects.
    'start' is the datetime to fetch from
    'end' is the datetime  to fetch to
    """          
    end = end or datetime.date.today()
   
    failures = []
    stats = dict([(x.name, {}) for x in feeds])
    site_series_count = 0
    for feed_obj in feeds:
        feed = feed_obj.name
        try:    
            feed_obj.get_logger().setLevel(logLevel)
            feed_obj.parser.get_logger().setLevel(logLevel)
            count = feed_obj.go(start, end)
            if count == 0:
                msg = 'no data updates for feed=%s' % feed
                # todo: would be nice to know how many cache_files were downloaded
                log.warn(msg)
                failures.append(msg)
                stats[feed]['errors'] = msg
            elif feed_obj.expected_series_count and \
                count < feed_obj.expected_series_count:
                    msg = 'expected feed item count is %s but only processed '\
                      '%s.' % (feed_obj.expected_series_count,
                      count)
                    log.warn(msg)
                    stats[feed]['warning'] = msg
            stats[feed]['count'] = count
            stats[feed]['obj'] = feed_obj
            site_series_count += count

        except ErrorForAllRequests, fetch_ex:
            _, t, v, tbinfo = compact_traceback()
            msg = 'feed failure for {0}, errors are {1}. traceback is: ({2}:{3} {4})'.format(
              feed, str(fetch_ex), t, v, tbinfo)
            log.warn(msg)
            failures.append(msg)
            stats[feed]['errors'] = msg
        except Exception, ex:
            _, t, v, tbinfo = compact_traceback()
            msg = 'feed failure for {0}, errors are {1}. traceback is: ({2}:{3} {4})'.format(
              feed, str(ex), t, v, tbinfo)
            log.warn(msg)
            failures.append(msg)
            stats[feed]['errors'] = msg

    clean_stats = [(feed, feed_stats) for (feed, feed_stats) in\
                   stats.iteritems() if 'errors' not in feed_stats]
    
    def statistics_for(key):
        return zip(*[(feed, feed_stats) for (feed, feed_stats) \
            in stats.iteritems() if key in feed_stats])
    
    if len(clean_stats) > 0:
        log.info('statistics=%s' % str(clean_stats))
    if len(failures) > 0:
        err_msg = 'Run site=%s failed for %s feeds %s. Errors: %s'
        error_feeds = statistics_for('errors')
        if len(feeds) == len(failures):
            raise ErrorForAllFeeds(err_msg % (name, 'all',
                str(error_feeds[0]), str(error_feeds[1])))
        else:
            raise ErrorForSomeFeeds(err_msg % (name, 'some',
                str(error_feeds[0]), str(error_feeds[1])))

    return stats


