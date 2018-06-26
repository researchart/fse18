#!/usr/bin/env perl
# Author: Jamie Davis <davisjam@vt.edu>
# Description: Guess at the language a regex pattern matches
#
# Can be used as a distributed-map.pl workScript

use strict;
use warnings;

use JSON::PP;

# Check usage
if (scalar(@ARGV) ne 1) {
  die "Usage: $0 pattern-file.json\n";
}

# Arg parsing
my $file = $ARGV[0];
if (not -f $file) {
  die "Error, no such file <$file>\n";
}

# Read file and check validity
my $cont = `cat $file`;
chomp $cont;
my $patternObj = decode_json($cont);

my $pat = $patternObj->{pattern};
if (not defined($pat)) {
  die "Error, patternObj had no field 'pattern'\n  object: " . encode_json($patternObj) . "\n";
}

# Prepare the list of valid languages and the mapping from language to "pretty print of the language"
# "pretty print" is used by run-tests.pl to compare test filenames to the resulting languages.
my @LANGUAGES = ("EMAIL", "UNIX_PATCH", "ERROR_MESSAGE", "TIME_WITH_UNITS", "ERROR_MESSAGE", "FILE", "FILETYPE", "HTML", "HTTP_REQUEST", "IPV4", "JS", "METACHARACTERS", "NUMBER", "HEX", "NUMERIC_SEQUENCE", "SHEBANG", "TAG", "UNIX_PATCH", "URL", "WHITESPACE", "EMPTY_STRING", "DATA_URI", "KEY", "USER_AGENT", "NAMING_CONVENTION"); 
my %LANGUAGES;
for my $lang (@LANGUAGES) {
  my $pretty = lc $lang;
  $pretty =~ s/_/-/g;
  $LANGUAGES{$lang} = $pretty; 
}

# Great, let's guess the language 
my @matchingLangs; # List of matched languages. Can contain duplicates, we set-ify later. TODO This should be flags, one per language, that's what we turn it into anyway.

###
# error message
###

# No error/failure? No problem!
my @negatives = ("no", "without");
my $ord_negatives = join("|", @negatives);
if ($pat =~ m/($ord_negatives) error/i or
    $pat =~ m/($ord_negatives) failure/i
){
}
elsif (300 < length($pat)) { # No error message is this long
}
else {
  if ($pat =~ m/\berr(or)?([^\\\.\|\w]|$)/i or
      $pat =~ m/WARNING/ or
      $pat =~ m/ERROR/ or
      $pat =~ m/EXPIRED/ or
      $pat =~ m/ENO[A-Z]{3,}/ or # UNIX errno
      $pat =~ m/\b[A-Z]\w+Error/ or # A javascript error like TypeError or AssertionError
      $pat =~ m/\bsorry\b/i or
      $pat =~ m/\bplease provide\b/i or
      $pat =~ m/\bexpected[^\\\.]/i or
      $pat =~ m/\bunexpected\b/i or
      $pat =~ m/\bexpect.*\bto be\b/i or
      $pat =~ m/\bexpect.*\breceived\b/i or
      $pat =~ m/\brequire.*\bto be\b/i or
      $pat =~ m/\brequired for\b/i or
      $pat =~ m/\bis required\b/i or
      $pat =~ m/\brequired\b/i or # subsumes the two above
      $pat =~ m/\brequires\b.*\bbut\b/i or
      $pat =~ m/\bmust\b/i or
      $pat =~ m/\bincorrect\b/i or
      $pat =~ m/\battempted\b.*\bused\b/i or
      $pat =~ m/\bwrong[^\\\.]/i or
      $pat =~ m/\bunbalanced\b/i or
      $pat =~ m/\btried\b.*\bbut\b/i or
      $pat =~ m/\bfail(ure|ed)?\b/i or
      $pat =~ m/\bnot supported\b/i or
      $pat =~ m/\bdon\\?'t\b/i or
      $pat =~ m/\bunsupported\b/i or
      $pat =~ m/\bunknown option\b/i or
      $pat =~ m/\bmismatch\b/i or
      $pat =~ m/\bnot equal\b/i or
      $pat =~ m/\bcould not\b/i or
      $pat =~ m/\bare not\b/i or
      $pat =~ m/\bcan( )?not\b/i or
      $pat =~ m/\bcan\\?'t\b/i or
      $pat =~ m/\bunable to\b/i or
      $pat =~ m/\bpremature\b/i or
      $pat =~ m/\bnot allowed\b/i or
      $pat =~ m/\bshould contain\b/i or
      $pat =~ m/\bshould be\b/i or
      $pat =~ m/\bmust be\b/i or
      $pat =~ m/\bmust (each|both|all) be\b/i or
      $pat =~ m/\bbut got\b/i or
      $pat =~ m/\bunrecogni[sz]ed\b/i or
      $pat =~ m/\bnot recogni[sz]ed\b/i or
      $pat =~ m/\bmissing\b/i or
      $pat =~ m/`undefined`/i or
      $pat =~ m/\bincompatible\b/i or
      $pat =~ m/\bnot exist(s|ed)?\b/i or
      $pat =~ m/\bnon-exist(e|a)nt\b/i or
      $pat =~ m/\balready exists?\b/i or
      $pat =~ m/\bmust exists?\b/i or
      $pat =~ m/\bno such\b/i or
      $pat =~ m/\b[nN]o[sS]uch\w+\b/ or
      $pat =~ m/\bnot defined\b/i or
      $pat =~ m/\bviolation\b/i or
      $pat =~ m/\b(not (a )?|[iI]n)valid(\b|[A-Z])/ or
      $pat =~ m/\b(requires|must be) a valid\b/ or
      $pat =~ m/\b\w+ value\b/ or # some criticism of a value
      $pat =~ m/\btoo many\b/i or
      $pat =~ m/\b(longer|more) than\b/i or
      $pat =~ m/\bshorter than\b/i or
      $pat =~ m/\bnot enough\b/i or
      $pat =~ m/\bnot an?\b/i or
      $pat =~ m/\bat least\b/i or
      $pat =~ m/\bshould (be|equal)\b/i or
      $pat =~ m/\bnot \w*\s*running\b/i or # not (currently? yet? etc.) running
      $pat =~ m/\bunknown \w+.?\s*\w+\b/i or # unknown NAME OF THING
      $pat =~ m/\bmust provide\b/i or # missing THING
      $pat =~ m/\bno.{0,5}(provided|given|found|available)\b/i or # missing THING
      $pat =~ m/\bno newline\b/i or # missing THING
      $pat =~ m/\bmust have\b/i or
      $pat =~ m/\bduplicate\b/i or
      $pat =~ m/\bneed to\b/i or
      $pat =~ m/\bout of \w+\b/i or # out of RESOURCE: bounds, memory, etc.
      $pat =~ m/\bread(-|\s|\\s)only\b/i or
      $pat =~ m/\bnot writ(e)?able\b/i or
      $pat =~ m/\b\w+ reserved\b/i or
      $pat =~ m/\balready (in use|used)\b/i or
      $pat =~ m/\btimed out\b/i or
      $pat =~ m/\balready closed\b/i or
      $pat =~ m/\bbad request\b/i or
      $pat =~ m/\bdisconnected\b/i or
      $pat =~ m/\babort\b/i or
      $pat =~ m/\bconflict\b/i or
      $pat =~ m/\btime( )?out\b/i or
      $pat =~ m/\btime limit\b/i or
      $pat =~ m/\b\w+ limit\b/i or # X limit
      $pat =~ m/\w+ (is|are):(\b|\s)/i or # invalid use of something: X is|are:
      $pat =~ m/\bUsage:/i # A usage message is an "error message" from the program they invoked
      ) {
    push @matchingLangs, $LANGUAGES{ERROR_MESSAGE};
  }
}

###
# email
###

# Emails have only one @ and at least one quantifier.
# TODO Actually emails can have more than one @. There should just one one @ *outside of char classes*.
if ($pat =~ m/@.*@/ or $pat !~ m/[\+\*{]/) {
}
# If that @ appears in a character class this is not an email.
elsif ($pat =~ m/\[[^]]*@[^]]*\]/) {
}
else {
  # In both cases we're looking for a quantified thing immediately preceding @. The quantifier should not be '\s*' or '\s+'.
  # General: has a quantifier near a possibly-escaped @, then later either: '.+' (simple) or a quantifier, eventually an escaped period, and another quantifier or a {
  if ($pat =~ m/(?<!\\s)(\+|\*)\\?@(\(?\\?\.\)?[\+\*]$|.*(\+|\*).*\\\..*(\+|\*|{[\d\\d,]+))/ or
      $pat =~ m/(?<!\\s)(\+|\*)\)?@\(?\w+\\?\.\w+\)?\$?$/ # looking for a specific domain, e.g. blah@gmail.com
  ) {
    push @matchingLangs, $LANGUAGES{EMAIL};
  }
}

###
# tag
###

# Tag: < ... >
if ($pat =~ m/^\^?<.*>\$?$/) {
  push @matchingLangs, $LANGUAGES{TAG};
}

###
# html
###

# https://www.w3schools.com/tags/ref_byfunc.asp
my @htmlTags = (# Basic HTML
                "!DOCTYPE", "html", "head", "title", "body", "h1", "h2", "h3", "h4", "h5", "h6", "p", "br", "hr", "!--",
                # Formatting
                "acronym", "abbr", "address", "b", "bdi", "bdo", "big", "blockquote", "center", "cite", "code", "del",
                "dfn", "em", "font", "i", "ins", "kbd", "mark", "meter", "pre", "progress", "q", "rp", "rt", "ruby", "s", "samp",
                "small", "strike", "strong", "sub", "sup", "template", "time", "tt", "u", "var", "wbr",
                # Forms and input
                "form", "input", "textarea", "button", "select", "optgroup", "option", "label", "fieldset", "legend", "datalist", "output",
                # Frames
                "frame", "frameset", "noframes", "iframe",
                # Images
                "img", "map", "area", "canvas", "figcaption", "figure", "picture",
                # Audio / Video
                "audio", "source", "track", "video",
                # Links
                "a", "link", "nav",
                # Lists
                "ul", "ol", "li", "dir", "dl", "dt", "dd", "menu", "menuitem",
                # Tables
                "table", "caption", "th", "tr", "td", "thead", "tbody", "tfoot", "col", "colgroup",
                # Styles and semantics
                "style", "div", "span", "header", "footer", "main", "section", "article", "aside", "details", "dialog", "summary", "data",
                # Meta Info
                "head", "meta", "base", "basefont",
                # Programming
                "script", "noscript", "applet", "embed", "object", "param",
               );

for my $tag (@htmlTags) {
  # After an html tag could be whitespace or a >, but certainly not a word
  # They might be capturing the tag name so permit an opening (
  if ($pat =~ m/<\(?$tag\W/ or # opening tag
      $pat =~ m/<\\\/\(?$tag\W/ # closing tag
  ) {
    push @matchingLangs, $LANGUAGES{HTML};
    last;
  }
}

# any OR'd sequences of tag names?
pos $pat = undef; # reset the /g flag
while ($pat =~ m/\((.*?)\)/g) {
  my %uniqueHTMLTags;
  my @fields = split(/\|/, $1); # case sensitive, don't allow SECTION since I don't think that's standard HTML.

  $fields[0] =~ s/^\W+//; # Trim leading characters e.g. (?:xxx|...)
  for my $f (@fields) {
    if (&contains(\@htmlTags, $f)) {
      $uniqueHTMLTags{$f} = 1;
    }
  }

  my $nUniqueHTMLTags = scalar(keys %uniqueHTMLTags);
  if (3 <= $nUniqueHTMLTags) {
    my @tags = keys %uniqueHTMLTags;
    print STDERR "tag match: @tags\n";
    push @matchingLangs, $LANGUAGES{HTML};
    last;
  }
}

# This is a gimme. Probably other tags too?
if ($pat =~ m/^href=/) {
  push @matchingLangs, $LANGUAGES{HTML};
}

###
# http-request
###

# https://developer.mozilla.org/en-US/docs/Web/HTTP/Methods
my @HTTP_OPS = ("get", "head", "post", "put", "delete", "connect", "options", "trace", "patch");
my $ord_http_ops = join("|", @HTTP_OPS);

# Any or'd pair of HTTP operations, e.g. 'get|post' or 'HEAD|PATCH'.
pos $pat = undef; # reset the /g flag
while ($pat =~ m/($ord_http_ops)\|($ord_http_ops)/gi) {
  my ($lc_match1, $lc_match2) = (lc $1, lc $2);

  next if ($lc_match1 eq $lc_match2);
  if (&contains(\@HTTP_OPS, $lc_match1) and &contains(\@HTTP_OPS, $lc_match2)) {
    push @matchingLangs, $LANGUAGES{HTTP_REQUEST};
    last;
  }
}

###
# js
###

# If regex contains too many english-looking things in a row, probably a lengthy error message instead.
if ($pat =~ m/(\w+\s+){3,}/) {
}
else {
  my @JS_OBJECT_TYPES = ("Array", "Object", "Date", "String", "Math", "Undefined", "Null", "Function");
  my $ord_js_object_types = join("|", @JS_OBJECT_TYPES);

  my @REQUIRE_WORDS = ("require", "import");
  my $ord_require_words = join("|", @REQUIRE_WORDS);

  # I guess some of these could belong to many languages. We're targeting here JavaScript and things that transpile thereto -- coffeescript, livescript, and friends 
  my @JS_RESERVED_WORDS = ("if", "else", "else if", "switch", "case", "for", "do", "break", "class", "function", "function(\\\*\?)?", "var", "let", "const", "typeof", "private", "public", "true", "false", "null", "NaN", "undefined");
  my $ord_js_reserved_words = join("|", @JS_RESERVED_WORDS);

  # matching a function representation: maybe some kind of quantifier (usually \s*), then 'function', followed by some escaped whitespace char, possibly in a character class
  if ($pat =~ m/[\+\*]?.*function((\\s|\\n|\\r|\\t|\\v|\\f| )|\[[\\strn ]+\])[\+\*]?/) {
    push @matchingLangs, $LANGUAGES{JS};
  }
  # matching a 'function*' representation (generator)
  elsif ($pat =~ m/[\+\*]?.*function\(?\\\*\)?/) {
    push @matchingLangs, $LANGUAGES{JS};
  }
  # A line beginning with 'function' is a gimme
  elsif ($pat =~ m/^\^?function\b/) {
    push @matchingLangs, $LANGUAGES{JS};
  }
  # matching a 'use strict' in a JS file
  # could be in single or double quotes and could be using a character class to identify those quotes
  elsif ($pat =~ m/('|"|('|")\]\)?)use strict/) {
    push @matchingLangs, $LANGUAGES{JS};
  }
  # matching the output of toString on a particular object
  elsif ($pat =~ m/\[object ($ord_js_object_types)/) {
    push @matchingLangs, $LANGUAGES{JS};
  }
  # any ord pair of js object types
  elsif ($pat =~ m/($ord_js_object_types)\|($ord_js_object_types)/) {
    push @matchingLangs, $LANGUAGES{JS};
  }
  # parsing file for 'module exports'
  elsif ($pat =~ m/module\\?\.exports(\s|\\s)+=/) {
    push @matchingLangs, $LANGUAGES{JS};
  }
  # parsing file for generic 'require'
  elsif ($pat =~ m/require(\\s[\+\*]?)?\\?\(\\?(\\s[\+\*]?)?[\[\\'"]/) {
    push @matchingLangs, $LANGUAGES{JS};
  }
  # parsing file for specific 'require'-ish
  elsif ($pat =~ m/(import|require) \\?['"]/) {
    push @matchingLangs, $LANGUAGES{JS};
  }
  # parsing file for multiple 'require'-ish phrases
  elsif ($pat =~ m/($ord_require_words)\W+($ord_require_words)/) {
    push @matchingLangs, $LANGUAGES{JS};
  }
  # import from -- typescript?
  elsif ($pat =~ m/import\\s[\*\+].*from/) {
    push @matchingLangs, $LANGUAGES{JS};
  }
  # Call to core object's prototype
  elsif ($pat =~ m/($ord_js_object_types)\\?\.\w*[pP]rototype/) {
    push @matchingLangs, $LANGUAGES{JS};
  }
  # Looking for language reserved_words as though a parser
  elsif ($pat =~ m/\^(\((\?:)?)?($ord_js_reserved_words)\s*\)?\??\(?\\s[\*\+]/) {
    push @matchingLangs, $LANGUAGES{JS};
  }
  # Seems pretty JS-specific
  elsif ($pat =~ m/\btypeof\b/) {
    push @matchingLangs, $LANGUAGES{JS};
  }
  # try-catch code
  elsif ($pat =~ m/catch.*\(err/) {
    push @matchingLangs, $LANGUAGES{JS};
  }
  # variable assignment
  elsif ($pat =~ m/\[[\w\-]+\][\+\*](\s*|\\s[\+\*])=(\s*|\\s[\+\*])/) {
    push @matchingLangs, $LANGUAGES{JS};
  }

  # Any groups containing many OR'd reserved words?
  pos $pat = undef; # reset the /g flag
  while ($pat =~ m/\(([^\(\)]+?)\)/g) {
    my %uniqueReservedWords;
    my @fields = split(/\|/, $1);
    $fields[0] =~ s/^\W+//; # remove leading (?: etc. in (?:a|b|...)
    for my $f (@fields) {
      if (&contains(\@JS_RESERVED_WORDS, $f)) {
        $uniqueReservedWords{$f} = 1;
      }
    }

    my $nUniqueReservedWords = scalar(keys %uniqueReservedWords);
    if (3 <= $nUniqueReservedWords) {
      push @matchingLangs, $LANGUAGES{JS};
      last;
    }
  }

}

###
# UNIX patch
###

# ^(---|+++
if ($pat =~ m/^\^\((\\\-|\-){3}\|(\\\+|\+){3}/) {
  push @matchingLangs, $LANGUAGES{UNIX_PATCH};
}
# ^(+++|---
elsif ($pat =~ m/^\^\((\\\+|\+){3}\|(\\\-|\-){3}/) {
  push @matchingLangs, $LANGUAGES{UNIX_PATCH};
}

###
# whitespace
###

# Contains at least one whitespace char, possibly in a group or a character class, possibly disjuncted or quantified or in character classes, perhaps with beginning/end anchors.
if ($pat =~ /\\s|\\n|\\r|\\t|\\v|\\f| / and $pat =~ m/^\^?(\\s|\\n|\\r|\\t|\\v|\\f| |[\|\*\+\[\]\(\)]|{\d?,\d?})+\$?$/) {
  push @matchingLangs, $LANGUAGES{WHITESPACE};
}
# Pattern contains only real whitespace characters (e.g. spaces or tabs or newlines), possibly with ^/$ anchors.
elsif ($pat =~ m/^\^?\s+\$?$/) {
  push @matchingLangs, $LANGUAGES{WHITESPACE};
}

###
# empty-string
###

if ($pat =~ m/^\^\$$/) {
  push @matchingLangs, $LANGUAGES{EMPTY_STRING};
}

###
# data-uri
###

# All the data URIs I've seen look something like this
if ($pat =~ m/\^.*data:.*base64.*\$$/) {
  push @matchingLangs, $LANGUAGES{DATA_URI};
}

###
# key
###

# e.g. 'PRIVATE KEY---'
if ($pat =~ m/(\s|\\s)KEY(\s|\\s|\\?-)/) {
  push @matchingLangs, $LANGUAGES{KEY};
}

###
# user-agent
###

#{"pattern":"SamsungBrowser.+Mobile VR"}
#{"pattern":"(Android|webOS|iPhone|iPad|iPod|BlackBerry|IEMobile|Opera Mini)"}
#{"pattern":"Android 2[\\s\\S]2"}
#{"pattern":"(Samsung)(SGH)(i[0-9]+)"}
#{"pattern":"iPhone|iP[ao]d"}
#{"pattern":"\\b(Android|Windows Phone|webOs)\\b"}
#{"pattern":"^Mozilla\\/[\\s\\S]*Gecko[\\s\\S]*lolifox\\/([0-9\\.]+)"}
#{"pattern":"^LinkExaminer\\/([0-9\\.]+) \\(Windows\\)$"}
#{"pattern":"^GSiteCrawler\\/([0-9a-z\\.]+)"}
#{"pattern":"Googlebot/\\d+.\\d+"}
#{"pattern":"mozilla[\\s\\S]*applewebkit[\\s\\S]*DeskBrowse\\/([0-9a-z\\+\\-\\.]+)[\\s\\S]*"}
#{"pattern":"^Mozilla[\\s\\S]*Postbox\\/([0-9a-zA-Z\\.]+)"}
#{"pattern":"iPod"}
#{"pattern":"VMS_Mosaic\\/([0-9.]*)"}
#{"pattern":"MIDP|SymbianOS|NOKIA|SAMSUNG|LG|NEC|TCL|Alcatel|BIRD|DBTEL|Dopod|PHILIPS|HAIER|LENOVO|MOT-|Nokia|SonyEricsson|SIE-|Amoi|ZTE"}
#{"pattern":"(Opera Mini)|Kindle|webOS|BlackBerry|(Opera Mobi)|(Windows Phone)|IEMobile"}
#{"pattern":"^Mozilla[\\s\\S]*iPhone[\\s\\S]*AppleWebKit[\\s\\S]*CriOS\\/([0-9\\.]+)[\\s\\S]*Mobile[\\s\\S]*Safari"}
#{"pattern":"mozilla[\\s\\S]*gecko\\/[0-9]+[\\s\\S]*k-ninja\\/([0-9a-z\\+\\-\\.]+)[\\s\\S]*"}
#{"pattern":"mozilla[\\s\\S]*Maxthon ([0-9a-z\\+\\-\\.]+)"}
#{"pattern":"iOS|iPhone OS|Android|BlackBerry|BB10|Series ?[64]0|J2ME|MIDP|opera mini|opera mobi|mobi.+Gecko|Windows Phone"}
#{"pattern":"(Windows NT 4.0|WinNT4.0)"}
#{"pattern":"(vivaldi|opera|chrome|safari|firefox|msie|edge(?=\\/)|trident(?=\\/))\\/?\\s*(\\d+)"}
#{"pattern":"chrome|opera"}
#{"pattern":"Chrome\\/([\\w\\W]*?)\\."}

my @userAgents = ("Samsung", "SamsungBrowser", "Android", "webOS", "iPhone", "iPad", "iPod", "BlackBerry", "IEMobile", "Opera Mini", "Opera Mobi", "Windows Phone", "Mozilla", "LinkExaminer", "GSiteCrawler", "Googlebot", "VMS_Mosaic", "MIDP", "SymbianOS", "NOKIA", "SAMSUNG", "LG", "NEC", "TCL", "Alcatel", "BIRD", "DBTEL", "Dopod", "PHILIPS", "HAIER", "LENOVO", "Nokia", "SonyEricsson", "Amoi", "ZTE", "Kindle", "Safari", "CriOS", "AppleWebKit", "iOS", "BB10", "Windows NT", "WinNT", "vivaldi", "msie", "trident", "chrome", "Chrome", "opera", "Opera", "BrowserNG", "OSRE", "Ovi", "Maemo");

for my $ua (@userAgents) {
  if ($pat =~ m/^\W{0,10}$ua($|\W(?!\s?\w+\s\w+))/i) { # useragent regexes have one of the user agents near the front, and shouldn't have a "normal sentence" after
    print STDERR "matched $ua\n";
    push @matchingLangs, $LANGUAGES{USER_AGENT};
    last;
  }
}

# In case they don't have parens around a disjunction, just check for a bunch of user-agent strings.
my $nRawMatched = 0;
for my $ua (@userAgents) {
  if ($pat =~ m/$ua/) {
    $nRawMatched++;
  }
}
if (5 < $nRawMatched) {
  push @matchingLangs, $LANGUAGES{USER_AGENT};
}

# any OR'd sequences of multiple user agents?
pos $pat = undef; # reset the /g flag
while ($pat =~ m/\((.*?)\)/g) {
  print STDERR "match: $1\n";
  my %uniqueUserAgents;
  my @fields = split(/\|/, $1);

  $fields[0] =~ s/^\W+//; # Trim leading characters e.g. (?:xxx|...)
  my @lcAgents = map { lc $_ } @userAgents;
  my @lcFields = map { lc $_ } @fields;

  for my $f (@lcFields) {
    if (&contains(\@lcAgents, $f)) {
      $uniqueUserAgents{$f} = 1;
    }
  }

  my $nUniqueUserAgents = scalar(keys %uniqueUserAgents);
  if (3 <= $nUniqueUserAgents) {
    my @uas = keys %uniqueUserAgents;
    print STDERR "ua match: @uas\n";
    push @matchingLangs, $LANGUAGES{USER_AGENT};
    last;
  }
}

###
# naming-convention
###

# All naming conventions have begin and end anchors
if ($pat !~ m/^\^.+\$$/) {
}
# Naming conventions are not too long
elsif (100 < length($pat)) {
}
else {
  # All character classes should contain only a-zA-Z0-9\d\-\_$
  my $couldBeNamingConvention = 1;
  pos $pat = undef; # reset the /g flag
  while ($pat =~ m/\[([^\]]+)\]/g) {
    my $classMembers = $1;
    if ($classMembers !~ m/^[azAZ09\\d\-_\$]+$/) {
      $couldBeNamingConvention = 0;
      last;
    }
  }

  # All characters *not* in character class must be "special"
  my @charsNotInCharClasses;

  # Parser
  my $containsAtLeastOneCharClass = 0;
  my $inCharClass = 0;
  my $escaped = 0;
  for my $i (0 .. length($pat) - 1) {
    my $char = substr($pat, $i, 1);

    # In char class, skip until we get out. Watch out for escape sequences here.
    if ($inCharClass) {
      # Skip if this char was escaped
      if ($escaped) {
        $escaped = 0;
        next;
      }
      if ($char eq "\\") {
        # Enter 'escaped' state
        $escaped = 1;
        next;
      }

      if ($char eq "]") {
        $inCharClass = 0;
      }
      next;
    }
    # Not in char class: consume characters until we enter
    else {
      # Skip if this char was escaped
      if ($escaped) {
        $escaped = 0;
        push @charsNotInCharClasses, $char;
        next;
      }
      if ($char eq "\\") {
        # Enter 'escaped' state
        $escaped = 1;
        push @charsNotInCharClasses, $char;
        next;
      }

      if ($char eq "[") {
        $containsAtLeastOneCharClass = 1;
        $inCharClass = 1;
        next;
      }

    }
  }

  if (not $containsAtLeastOneCharClass) {
    print STDERR "NAMING_CONVENTION: No character classes, so not naming convention\n";
    $couldBeNamingConvention = 0;
  }

  my $charsNotInCharClasses = join("", @charsNotInCharClasses);
  if ($charsNotInCharClasses !~ m/^[\^\$!\+\*\-\(\?:\)\\wd\._\|]*$/) { # all manner of special chars, including char classes like \w \d
    print STDERR "NAMING_CONVENTION: Unexpected chars in <$charsNotInCharClasses>\n";
    $couldBeNamingConvention = 0;
  }

  if ($couldBeNamingConvention) {
    push @matchingLangs, $LANGUAGES{NAMING_CONVENTION};
  }
}

###
# metacharacters
###

# This is experimental. Might be too generous.

# Can't be metacharacters if:
if ($pat =~ m/(?<!\\u)(?:[a-tv-zA-Z0-9]{4,})/ or # it has too many "normal" characters in a row that aren't part of a unicode character '\uXXXX'
    $pat =~ m/{[\d,]+}/) # it has quantified groups in it
{
  # metacharacters might be present but only as part of a more complex language.
  # calling this regex 'metacharacters' would be an oversimplification.
}
else {
  # Check for a character class containing metachars.
  pos $pat = undef; # reset the /g flag
  while ($pat =~ m/\[([^\]]+)\]/g) {
    my $classContents = $1;

    if ($classContents =~ m/a\-z/ or $classContents =~ m/A\-Z/ or $classContents =~ m/0\-9/) {
      # If the class includes these common ranges, it is not matching *just* metachars
      next;
    }

    # Some metachars: !#$%&'*+/=?^
    my %metaChars = ("!" => 0, "#" => 0, "\$" => 0, "%" => 0, "&" => 0, "'" => 0, "*" => 0, "+" => 0, "/" => 0, "=" => 0, "?" => 0, "^" => 0);
    my $nMetaChars = 0;

    # How many metachars were in this character class?
    for my $char (split(//, $classContents)) {
      if (defined($metaChars{$char})) {
        $metaChars{$char}++;
      }
    }

    my $nUniqueMetaChars = grep { $metaChars{$_} } keys %metaChars;

    # If plenty, call it 'metacharacters'.
    if (5 < $nUniqueMetaChars) {
      push @matchingLangs, $LANGUAGES{METACHARACTERS};
      last;
    }
  }
}

###
# IPv4 (a specific, generally recognizable type of numeric-sequence)
###

# If a pattern has a series of .-separated, quantified \d's, it's probably an IPv4 address.
# Look for \d+\.\d+\.\d+\. or \.\d+\.\d+\.\d+. They might specify [0-9] instead of \d.
# They might try to capture one of the segments.
# If they quantify, it should be {3}
if ($pat =~ m/(\(?(\\d|\[0-9\])(\+|\*|\{3\})\)?\\?\.\)?){3,}/ or 
    $pat =~ m/(\(?\\?\.(\\d|\[0-9\])(\+|\*|\{3\})\)?){3,}/)
{
	push @matchingLangs, $LANGUAGES{IPV4};
}

###
# numeric-sequence
###

# Can't be too long.
# If a pattern encodes a lot of sequences of digits with a specified length,
# this is recipe for some kind of man-made number-based code (zip code, phone number, CC number, etc.).
if ($pat =~ m/((\\d|\[0-9\])\{.*){3,}/) {
	push @matchingLangs, $LANGUAGES{NUMERIC_SEQUENCE};
}

###
# Number
###

# Check for an encoding of "a number" like 123 or 123.456, etc.

if ($pat =~ m/[a-zA-Z]{3,}/) {
	# If the regex explicitly captures a number it will not have long runs of characters in a row.
}
# This might be too conservative, we'll see.
#	elsif ($pat !~ m/^\^/ or $pat !~ m/\$$/) {
#		# Number regexes should have both anchors.
#	}
else { # Hmm, might be a number.
  # simple: an optional negative sign '-?', maybe a (, then a quantified character class permitting '\d or . or , or 0, -, and 9'
  #         NB we guess the character class contents and permit them in any order, hence the nested \[[contents]+\] pattern
  if ($pat =~ m/\-\?\(?\[[\\d,\.0\-9]+\][\+\*]/) {
    push @matchingLangs, $LANGUAGES{NUMBER};
  }
  # variation on simple: a signed digit, like: [-+]?(?\d
  elsif ($pat =~ m/\[(\-\+|\\-\\+|\+\-|\\+\\-)\]\??(\()?\\d/) {
    push @matchingLangs, $LANGUAGES{NUMBER};
  }
  # an amount in USD
  elsif ($pat =~ m/\[(\-\+|\\-\\+|\+\-|\\+\\-)\]\??(\\s[\-\+]*)?\\\$/) {
    push @matchingLangs, $LANGUAGES{NUMBER};
  }
	# Something less sophisticated:
  #   - looking for something like \d+\.\d+
  #   - anchored
	#   - no alphabet chars but d
  #   - must not be too long
	elsif ($pat =~ m/\\d[\+\*]\\\.\??\\d[\+\*].*/ and
	       $pat =~ m/^\^.*\$$/ and
	       $pat !~ m/[a-ce-zA-Z]/ and
	       length($pat) < 25)
  {
    push @matchingLangs, $LANGUAGES{NUMBER};
	}
  # Expression of a possibly-large decimal number: 123,456.78 (US) or 123.456,78 (European)
  # This has too many false positives. There are many reasons to have \d+\.\d+.
  ## Pretty common to have something like \d+\.\d+ to check for numbers (or IP addresses, hence the extra check for something like an IPv4 address)
  ## In European style the decimal place is indicated with a ,
  #elsif ($pat =~ m/\\d[\+\*](,|\\\.)\??\(?\\d[\+\*]/ and $pat !~ m/\\d\+\\\.\\d\+\\\./) {
  #	print "3\n";
  #  push @matchingLangs, $LANGUAGES{NUMBER};
  #}
}

###
# hex
###


if ($pat =~ m/(?<!0)0x[\(\)\?:\\d\d\-\[\]]*(a\-f|abcdef)/i or # 0x followed shortly by 'a-f', not case sensitive
    $pat =~ m/(?<!0)0x[\(\)\?:]*\\d/ or # 0x followed immediately (minus punctuation) by \d
    $pat =~ m/^\^?0x/ # 0x at the beginning of the string
) {
  push @matchingLangs, $LANGUAGES{HEX};
}

###
# time-with-units
###

# Look for a possibly-quantified number followed by a group (a|b|c) or a char class [abc] with plenty of time units
pos $pat = undef; # reset the /g flag
while ($pat =~ m/\\d[\+\*]?\)?(?:\s*|\\s[\+\*])?(\[(?:[^\]]+)\]|\((?:[^\)])+\))/g) {
  print STDERR "maybe time\n";
  my $possibleTimeUnits = $1;

  my @TIME_UNITS = ("s", "m", "h", "d", "w", "seconds", "minutes", "hours", "days", "weeks", "months");
  my %timeUnitsPresent;

  if ($possibleTimeUnits =~ m/^\[(.*)\]$/) {
    my $classContents = $1;
    my @classContents = split(//, $classContents);

    for my $char (@classContents) {
      my $lc_char = lc $char; # Permit m | M, etc.
      if (&contains(\@TIME_UNITS, $lc_char)) {
        $timeUnitsPresent{$lc_char} = 1;
      }
    }
  }
  elsif ($possibleTimeUnits =~ m/^\((.*)\)$/) {
    my $groupContents = $1;
    my @groupContents = split(/\|/, $groupContents);
    $groupContents[0] =~ s/^\W+//; # remove leading (?: etc. in (?:a|b|...)

    for my $member (@groupContents) {
      my $lc_member = lc $member; # Permit m | M, etc.
      if (&contains(\@TIME_UNITS, $lc_member)) {
        $timeUnitsPresent{$lc_member} = 1;
      }
    }
  }

  my @timeUnitsPresent = keys %timeUnitsPresent;
  my $nUniqueTimeUnits = scalar(keys %timeUnitsPresent);
  print STDERR "time units present in <$possibleTimeUnits>: <@timeUnitsPresent>\n";

  if (3 <= $nUniqueTimeUnits) {
    push @matchingLangs, $LANGUAGES{TIME_WITH_UNITS};
    last;
  }
}

###
# url
###

my $protocols = "http|https|ftp|otpauth|ssh|git|bitid|irc|mailto|rmtp|mms";
# simple: does it have a recognizable protocol followed by a : in the first few chars?
# Ensure the : is not part of a '(?:' sequence using a lookbehind assertion: (?<! ... )
#   (we repeat this trick below)
if ($pat =~ m/^.{0,5}($protocols).{0,5}(?<!\(\?):/) {
  push @matchingLangs, $LANGUAGES{URL};
}
# medium: does it have some text followed by :// ?
if ($pat =~ m/.+(?<!\(\?):[\]\)\?\*\+]*(\\?\/\\?\/|\\?\/\{2\})/) {
  push @matchingLangs, $LANGUAGES{URL};
}
# more complex: near the beginning, is there a recognizable protocol followed vaguely soon by // ?
elsif ($pat =~ m/^.{0,5}($protocols).{0,50}\\?\/\\?\//) {
  push @matchingLangs, $LANGUAGES{URL};
}

###
# shebang
###

if ($pat =~ m/\^\#\!/) {
  push @matchingLangs, $LANGUAGES{SHEBANG};
}

###
# filetype / file
###

# If they are looking for a member of a group of file endings,
# there will be a list of the form (a|b|c|...); one of them has to be the *last* one so that's what we're trying to capture here: e.g. '|jpg'
# We hope that the last one isn't something too fancy, though we can handle something like 'docx?'
#   analyze-regexps/language/guess-patterns-language.pl analyze-regexps/language/test/filetype/filetype-8.json 
my @endings = ("mp3", "mp4", "wav", "m4a", "flac", "ogg", "webm", "png", "svg", "gif", "jpg", "jpeg", "js", "jsx", "ls", "ts", "jade", "json", "eot", "appcache", "swf", "webp", "woff", "woff2", "ttf", "css", "htm", "html", "shtml", "pdf", "doc", "docx", "xls", "xlsx", "ppt", "pptx", "txt", "rtf", "md", "markdown", "mdown", "mkdn", "mkd", "mdwn", "mdtxt", "mdtext", "text", "mustache", "sass", "scss", "xml", "xml.rels", "nunjucks", "tpl");
my $or_endings = join("|", @endings);
my $maybe_dot_or_endings = join("|", map { "\\\\?\.$_" } @endings);
#print "maybe_dot_or_endings /$maybe_dot_or_endings/\n";
#print "pattern <$pat>\n";
if ($pat =~ m/\|($or_endings)\??\)*\??\$?$/i or
    $pat =~ m/\|($maybe_dot_or_endings)\??\)*\??\$?$/i)
{
  push @matchingLangs, $LANGUAGES{FILETYPE};
}
# If they are looking for text leading up to *one* of these endings, then the end of the pattern, they probably want a specific file.
# they might be careless and not escape the . in e.g. 'test.js' vs. 'test\.js'
elsif ($pat =~ m/(^|[\^\(\)\w_\-\+\*\\\|])\\?\[?\.\]?($or_endings)\)*\$?$/i) {
	push @matchingLangs, $LANGUAGES{FILE};
}

# Maybe more than one match, but in some cases there's a hierarchy.
@matchingLangs = &applyMatchingLangsHierarchy(@matchingLangs);

# Summary

if (scalar(@matchingLangs) eq 0) {
  $patternObj->{language} = "UNKNOWN-NONE";
}
elsif (scalar(@matchingLangs) eq 1) {
  $patternObj->{language} = $matchingLangs[0];
}
else {
  print STDERR "Got more than one match: <@matchingLangs>\n";
  $patternObj->{language} = "UNKNOWN-MANY-@matchingLangs";
}
print STDERR "Language: $patternObj->{language}\n";

# Emit
print STDOUT encode_json($patternObj) . "\n";

########################################################
# Helpers
########################################################

sub applyMatchingLangsHierarchy {
  my (@ml) = @_;

  my %set;
  map { $set{$_} = 1 } @ml;
  @ml = keys %set;

  if (@ml) {
    print STDERR "Matched languages: <@ml>. Applying hierarchy.\n";
  }
  else {
    print STDERR "Matched no languages\n";
    return @ml;
  }

  # Which langs did we match?
  my %langs;
  for my $l (@ml) {
    $langs{$l} = 1;
  }

  # Apply hierarchy.

  # Tag is the strongest match.
  # A regex could look for many things inside a <tag>, but the overall language is $LANGUAGES{TAG}.
  # Unless the tag is also HTML!
  if ($langs{$LANGUAGES{TAG}}) {
    if ($langs{$LANGUAGES{HTML}}) {
      return ($LANGUAGES{HTML});
    }
    else {
      return ($LANGUAGES{TAG});
    }
  }

  # empty-string is also quite strong.
  if ($langs{$LANGUAGES{EMPTY_STRING}}) {
    return ($LANGUAGES{EMPTY_STRING});
  }

  # as is shebang
  if ($langs{$LANGUAGES{SHEBANG}}) {
    return ($LANGUAGES{SHEBANG});
  }

  # very strong, error message is, but weaker than JS, HTML
  if ($langs{$LANGUAGES{ERROR_MESSAGE}}) {
    if ($langs{$LANGUAGES{JS}}) {
      return ($LANGUAGES{JS});
    }
    elsif ($langs{$LANGUAGES{HTML}}) {
      return ($LANGUAGES{HTML});
    }
    else {
      return ($LANGUAGES{ERROR_MESSAGE});
    }
  }

  # and filetype
  if ($langs{$LANGUAGES{FILETYPE}}) {
    return ($LANGUAGES{FILETYPE});
  }

  # A url is a strong match but might contain others as sub-classes (e.g. metacharacters)
  # When URL and HTML overlap, keep them both
  if ($langs{$LANGUAGES{URL}} and not $langs{$LANGUAGES{HTML}}) {
    return ($LANGUAGES{URL});
  }

  # Email trumps metacharacters
  if ($langs{$LANGUAGES{EMAIL}} and $langs{$LANGUAGES{METACHARACTERS}}) {
    return ($LANGUAGES{EMAIL});
  }

	# ipv4 trumps numberic-sequence and number.
  if ($langs{$LANGUAGES{IPV4}} and ($langs{$LANGUAGES{NUMERIC_SEQUENCE}} or $langs{$LANGUAGES{NUMBER}})) {
    return ($LANGUAGES{IPV4});
  }

	# Numeric sequence trumps number.
  if ($langs{$LANGUAGES{NUMERIC_SEQUENCE}} and $langs{$LANGUAGES{NUMBER}}) {
    return ($LANGUAGES{NUMERIC_SEQUENCE});
  }

  # time-with-units trumps html -- html shouldn't contain \d+
  if ($langs{$LANGUAGES{TIME_WITH_UNITS}} and $langs{$LANGUAGES{HTML}}) {
    return ($LANGUAGES{TIME_WITH_UNITS});
  }

  return @ml;
}

########################################################
# Utils
########################################################

# input: (\@L, $e)
# output: 1 if $e is in @L, else 0
sub contains {
  my ($L, $e) = @_;
  for my $l (@$L) {
    if ($e eq $l) {
      return 1;
    }
  }

  return 0;
}
