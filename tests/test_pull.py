import unittest

class TestFileListCriteria(unittest.TestCase):

    def _make(self, cache_location=None):
        return MyFiles(cache_location)

    def test_ctor(self):
        criteria = self._make()
        self.assertEqual(criteria.cache_location,
            FileListCriteria.temp_dir,
            "cache_location should be set to FileListCriteria.temp_dir.")
        
    def test_ctor_with_cache_loc(self):
        criteria = self._make('/foo/bar')
        self.assertEqual(criteria.cache_location,
            '/foo/bar',
            "cache_location should be set to my /foo/bar.")          
   
class TestFetch(unittest.TestCase):

    def _make(self, criteria):
        from pull import UrlProtocol
        return UrlProtocol(criteria)
    
    def test_url_fetch(self):
        def build_who_files(cache_location, start, end):
            url = "http://www.who.int/csr/don/archive/year/{0}/en/index.html"
            files = []
            for year in range(start.year, end.year+1):
                files.append((url.format(year),
                              cache_location+'{0}.html'.format(year)))
            return files     
            
        criteria = MyFiles(builder=build_who_files) 
        protocol = self._make(criteria)
        import datetime
        start = datetime.date(2011,1,1)
        end = datetime.date(2012,1,1)
        downloads = protocol.call(start, end)
        self.assertTrue(len(downloads)==2,
                        "should have downloaded 2 year files")
        self.assertEqual(downloads, [x[1] for x in criteria.build(start, end)],
            "protocol should have dowloaded files specified by criteria.")

    def test_url_fetch_error(self):
        def build_files(cache_location, start, end):
            return [('you_wont_find_this', 'x')]     
            
        criteria = MyFiles(builder=build_files) 
        protocol = self._make(criteria)
        import datetime
        from pull import ErrorForAllRequests
        d = datetime.date.today()
        self.assertRaises(ErrorForAllRequests, lambda: protocol.call(d, d))
        
    def test_url_fetch_warn(self):
        def build_files(cache_location, start, end):
            return [('you_wont_find_this', 'x'),
              ("http://www.who.int/csr/don/archive/year/2012/en/index.html",
                '/tmp/delme')]     
            
        criteria = MyFiles(builder=build_files) 
        protocol = self._make(criteria)
        import datetime
        d = datetime.date.today()
        results = protocol.call(d, d)
        self.assert_(len(results) == 1, 'should have dowloaded 1 file')  
               
class TestFeed(unittest.TestCase):
    
    def test_build(self):
        from pull import build_feed, SkipProtocol
        def build_files():
            return []
        criteria = MyFiles(builder=build_files)
        feed = build_feed('test', SkipProtocol(criteria)) 
        self.assert_(feed.name=='test', 'name not set')      
    
    def test_go(self):
        from pull import build_feed, SkipProtocol, go, ErrorForAllFeeds
        import datetime
        def build_files(*args):
            return ['x', 'y']
        criteria = MyFiles(builder=build_files)
        feed = build_feed('test', SkipProtocol(criteria)) 
        d = datetime.date.today()
        results = go('test_feed', [feed], d)
        self.assert_(results['test']['count']==2, 'should have 2 results')
        self.assertEqual(results['test']['obj'].updater.data_items,
           ['x', 'y'], 'updater should have stored what was input')              
        
    def test_go_no_results(self):
        from pull import build_feed, SkipProtocol, go, ErrorForAllFeeds
        import datetime
        def build_files(*args):
            return []
        criteria = MyFiles(builder=build_files)
        feed = build_feed('test', SkipProtocol(criteria)) 
        d = datetime.date.today()
        
        self.assertRaises(ErrorForAllFeeds,
                          lambda: go('test_feed', [feed], d))
    
    def test_go_fetch_error(self):
        def build_files(cache_location, start, end):
            return [('you_wont_find_this', 'x')]     
            
        criteria = MyFiles(builder=build_files) 
        import datetime
        from pull import UrlProtocol, build_feed, go, ErrorForAllFeeds
        protocol = UrlProtocol(criteria)
        feed = build_feed('test', protocol)
        from pull import ErrorForAllRequests
        d = datetime.date.today()
        self.assertRaises(ErrorForAllFeeds, lambda: go('test_feed', [feed], d))
        
    def test_go_some_feeds_failed(self):
        def build_bad_files(*args):
            return [('you_wont_find_this', 'x')]
        
        def build_ok_files(*args):
            return [
                ("http://www.who.int/csr/don/archive/year/2012/en/index.html",
                '/tmp/delme')]     
            
        import datetime
        from pull import UrlProtocol, build_feed, go, ErrorForSomeFeeds
        badfeed = build_feed('badfeed',
                             UrlProtocol(MyFiles(builder=build_bad_files)))
        okfeed = build_feed('okfeed',
                            UrlProtocol(MyFiles(builder=build_ok_files)))
      
        d = datetime.date.today()
        self.assertRaises(ErrorForSomeFeeds, lambda: go('test_feed',
            [badfeed, okfeed], d))    
        
    def test_go_with_parse(self):
        from pull import build_feed, SkipProtocol, go
        import datetime
        def build_files(*args):
            return ['x', 'y']
        
        from pull import Parser
        class MyParser(Parser):
            def parse(self, file_path):
                return ['x1', 'y1']
    
        criteria = MyFiles(builder=build_files)
        feed = build_feed('test', SkipProtocol(criteria), parser=MyParser()) 
        d = datetime.date.today()
        results = go('test_feed', [feed], d)
        self.assert_(results['test']['count']==2, 'should have 2 results')
        self.assertEqual(results['test']['obj'].updater.data_items,
           ['x1', 'y1'], 'updater should have stored what was input')        
     
from pull import FileListCriteria
class MyFiles(FileListCriteria):
    def __init__(self, cache_location=None, builder=None):
        FileListCriteria.__init__(self, cache_location=cache_location)
        self.builder = builder
        
    def build(self, start, end):
        if self.builder:
            return self.builder(self.cache_location, start, end)

