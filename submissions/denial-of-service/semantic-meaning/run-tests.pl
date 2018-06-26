#!/usr/bin/env perl
# Author: Jamie Davis <davisjam@vt.edu>
# Description: Run tests and report any mismatches

use strict;
use warnings;

use JSON::PP;
use File::Basename;

my $searchDir;
if (@ARGV) {
  $searchDir = $ARGV[0];
}

my @files = `find test/ -type f -name '*.json'`;
chomp @files;

# Filter by searchDir
if ($searchDir) {
  @files = grep { m/\/$searchDir\// } @files;
}

for my $file (@files) {
  my $cmd = "./guess-patterns-language.pl $file";
  print STDERR "$cmd\n";
  my $result = `$cmd 2>/dev/null`;
  chomp $result;

  my $json = decode_json($result);
  my $lang = $json->{language};

  my $basename = basename($file);
  my $should_be;
  my $should_not_be;
  if ($basename =~ m/^not-(.+)-\d/) {
    $should_not_be = $1;
    $should_not_be =~ s/^js-.*/js/;
    $should_not_be =~ s/^html-.*/html/;
    $should_not_be =~ s/^tag-.*/tag/;
  }
  elsif ($basename =~ m/^(.+)-\d/) {
    $should_be = $1;
    $should_be =~ s/^js-.*/js/;
    $should_be =~ s/^html-.*/html/;
    $should_be =~ s/^tag-.*/tag/;
  }

  if ($should_be and $lang !~ m/$should_be/) {
    print "Mismatch: file $file expected <$should_be> got <$lang>\n";
  }
  elsif ($should_not_be and $lang eq $should_not_be) {
    print "Mismatch: file $file should not be <$should_not_be> but got <$lang>\n";
  }
}
