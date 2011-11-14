
from datetime import datetime
import lxml.html
import re
import urlparse
import logging
import os
from feeds.config import Config
from pull.site import (
    FileListCriteria,
    Parser,
    Feed,
    UrlProtocol,
    Site
  )

# adapted from https://scraperwiki.com/scrapers/cdc_foodborne_outbreaks/edit/
class FoodParser(Parser):
    
    def cc(self, c,s):
        return len([x for x in s if x == c])
    
    def parse(self, file_path):
        with open(file_path) as f:
            html = f.read()
        page = lxml.html.fromstring( html )
        lis = page.cssselect('.main-inner ul li')
        l = []
        for li in lis:
            tc = li.text_content()
            if self.cc('-',tc) != 2:
                source,_,pathogen = tc.rpartition('-')
                if not source.strip():
                    if pathogen.find('-') > 0:
                        source,_,pathogen = pathogen.rpartition('-')
                    else:
                        continue
                href = urlparse.urljoin(FoodFile.url,
                           li.cssselect('a')[0].attrib.get('href') )
                l.append({'source':source, 'pathogen': pathogen, 'link': href})
        return l


class FoodFile(FileListCriteria):

    url = 'http://www.cdc.gov/outbreaknet/outbreaks.html'

    def __init__(self):
        FileListCriteria.__init__(self)

    def build(self, start, end):
        return [(FoodFile.url, 
                  self.cache_location+'outbreaks.html')]

class FoodOutbreaks(Feed):

    def __init__(self):
        Feed.__init__(self, UrlProtocol(FoodFile()), FoodParser())


class WhoFiles(FileListCriteria):

    url = "http://www.who.int/csr/don/archive/year/{0}/en/index.html"

    def __init__(self):
        FileListCriteria.__init__(self)

    def build(self, start, end):
        files = []
        for year in range(start.year, end.year+1):
            files.append((self.url.format(year),
                          self.cache_location+'{0}.html'.format(year)))
        return files   

# adapted from https://scraperwiki.com/scrapers/who_outbreaks/edit/
class WhoParser(Parser):
    
    months = ['January', 'February', 'March', 'April', 'May', 'June', 'July', 
              'August', 'September', 'October', 'November', 'December']
    
    def parse_date(self, datestr):
        m = re.match('^(\d+) (\w+) (\d+)', datestr).groups(0)
        return int(m[2]), datetime(year=int(m[2]), 
            month=WhoParser.months.index(m[1]) + 1, day=int(m[0]))
    
    def parse_item(self, s):
        m = re.match('.*-(.*) in (.*) -.*', s)
        if m:
            return m.groups(0)[0], m.groups(0)[1]
        m = re.match('.*-(.*) in (.*) \(.*', s)
        if m:
            return m.groups(0)[0], m.groups(0)[1]
        m = re.match('.*-(.*) in (.*)', s)
        if m:
            return m.groups(0)[0], m.groups(0)[1]
        # Changed in 2004
        m = re.match('(.*) [Ii]n (.*)', s)
        if m:
            return m.groups(0)[0], m.groups(0)[1]
        print '**', 'Failed to parse', s
        return s, ''

    
    def parse(self, file_path):
        with open(file_path) as f:
            html = f.read()
        page = lxml.html.fromstring( html )
        l = []
        url = WhoFiles.url.format(os.path.basename(file_path).rpartition('.')[0])
        lis = page.cssselect('.auto_archive li')
        for li in lis:
            href = li.cssselect('a')[0]
            link = urlparse.urljoin( url, href.attrib.get('href') )
            year,date = self.parse_date( href.text_content() )
            info = li.cssselect('.link_info')[0].text_content()
            disease, where = self.parse_item(info)
            disease = disease.strip()
            where = where.strip()
    
            if '-' in where:
                where = where[0:where.find('-')].strip()
            if u'-' in where:
                where = where[0:where.find(u'-')].strip()
            for x in [',',';',':',u'\u2013' ]:
                if x in where:
                    where = where[0:where.find(x)].strip()
                    disease = where[where.find(x)+1:].strip() + ' ' + disease
    
            d = {
                'year': year, 'date':date.isoformat(), 'link':link,
                'disease': disease.title(), 'where':where.title()
            }
            l.append(d)    
        return l            

class WhoOutbreaks(Feed):

    def __init__(self):
        Feed.__init__(self, UrlProtocol(WhoFiles()), WhoParser())

def main():
    logging.basicConfig(level=logging.DEBUG,
      format='%(asctime)s %(levelname)s %(name)s %(message)s')        
    d = datetime.today()    
    results = Site(Config).run('feeds.health', start=d, end=d)
    for feed, feed_result in results.iteritems():
        print 'feed={0}, count={1}'.format(feed, feed_result['count'])
        for item in feed_result['obj'].updater.data_items:
            print item 
   
if __name__ == "__main__":
    main()



      
